"""Software-only tests: fake UART, fake cameras, fake C++ child, real local HTTP."""
import http.client
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from carvision.transport import packet
from carvision.supervisor import Supervisor
from carvision.server import ControlServer


class FakeHardware:
    def __init__(self):
        self.opened = False
        self.writes = []
        self.fail = False

    def open(self):
        self.opened = True

    def write(self, steering, throttle):
        if self.fail:
            raise RuntimeError('fake UART failure')
        if not self.opened:
            raise RuntimeError('fake UART closed')
        self.writes.append((steering, throttle))

    def close(self):
        self.opened = False


class FakePreview:
    error = None
    def __init__(self):
        self.running = False
        self.fail_stop = False
    def start(self):
        self.running = True
    def stop(self):
        if self.fail_stop:
            raise RuntimeError('camera not released')
        self.running = False
    def frame(self, camera):
        return None


class FakeProcess:
    def __init__(self):
        self.returncode = None
        self.terminated = False
    def poll(self):
        return self.returncode
    def terminate(self):
        self.terminated = True
        self.returncode = 0
    def kill(self):
        self.returncode = -9
    def wait(self, timeout):
        return self.returncode


class SupervisorTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        (self.root / 'cpp/config').mkdir(parents=True)
        (self.root / 'cpp/build').mkdir()
        (self.root / 'cpp/config/race.json').write_text('{}')
        (self.root / 'cpp/build/smartcar').write_text('fake executable')
        self.now = 10.
        self.hardware = []
        self.preview = FakePreview()
        self.commands = []
        self.child = FakeProcess()
        def hardware_factory(_):
            hardware = FakeHardware()
            self.hardware.append(hardware)
            return hardware
        def popen(command, **kwargs):
            self.assertFalse(self.preview.running)
            self.assertTrue(all(not item.opened for item in self.hardware))
            self.commands.append(command)
            return self.child
        self.supervisor = Supervisor(self.root, hardware_factory=hardware_factory,
                                     preview=self.preview, popen=popen,
                                     clock=lambda: self.now, background=False)

    def tearDown(self):
        self.preview.fail_stop = False
        for hardware in self.hardware:
            hardware.fail = False
        self.supervisor.close()
        self.directory.cleanup()

    def action(self, name, **kwargs):
        return self.supervisor.action('owner', {'action': name, 'generation': self.supervisor.generation, **kwargs})

    def prepare(self):
        self.action('prepare', esc_off=True, supervised=True)

    def manual(self):
        self.prepare()
        self.action('enable', esc_on=True, stationary=True)

    def command(self, sequence=1, throttle=1570):
        self.action('command', sequence=sequence, throttle_us=throttle, steering_us=1700)

    def start(self, task='race'):
        return self.action('start', task=task, supervised=True, stationary=True, handoff_complete=True)

    def test_construction_status_and_get_never_open_uart(self):
        self.supervisor.status()
        self.supervisor.tick()
        self.assertEqual(self.hardware, [])

    def test_prepare_requires_true_not_truthy(self):
        with self.assertRaises(ValueError):
            self.action('prepare', esc_off='true', supervised=True)
        self.assertEqual(self.hardware, [])

    def test_prepare_and_enable_only_neutral(self):
        self.manual()
        self.supervisor.tick()
        self.assertTrue(all(t == 1500 for _, t in self.hardware[0].writes))

    def test_foreign_owner_cannot_drive_or_acquire_control(self):
        self.manual()
        for action in ('enable', 'command', 'start', 'prepare'):
            with self.assertRaises(PermissionError):
                self.supervisor.action('someone-else', {'action': action})
        self.assertEqual(self.supervisor.mode, 'manual')

    def test_new_operator_can_stop_lost_browser_but_cannot_drive(self):
        self.manual()
        self.command()
        self.supervisor.tick()
        self.supervisor.action('new-browser', {'action': 'stop'})
        self.assertEqual(self.supervisor.mode, 'idle')
        self.assertIsNone(self.supervisor.owner)
        self.assertEqual(self.hardware[0].writes[-1][1], 1500)
        self.assertFalse(self.hardware[0].opened)
        with self.assertRaises(ValueError):
            self.supervisor.action('new-browser', {'action': 'enable', 'esc_on': True,
                                   'stationary': True, 'generation': self.supervisor.generation})

    def test_late_manual_lease_goes_neutral(self):
        self.manual()
        self.command()
        self.supervisor.tick()
        self.assertEqual(self.hardware[0].writes[-1][1], 1570)
        self.now += 1.49
        self.supervisor.tick()
        self.assertEqual(self.hardware[0].writes[-1][1], 1570)
        self.now += .02
        self.supervisor.tick()
        self.assertEqual(self.hardware[0].writes[-1][1], 1500)
        self.assertEqual(self.supervisor.mode, 'prepared')
        with self.assertRaises(ValueError):
            self.command(sequence=2)

    def test_reordered_manual_command_does_not_revive_motion(self):
        self.manual()
        self.command(sequence=3, throttle=1500)
        with self.assertRaises(ValueError):
            self.command(sequence=2)
        self.supervisor.tick()
        self.assertEqual(self.hardware[0].writes[-1][1], 1500)

    def test_late_command_cannot_bypass_watchdog_tick(self):
        self.manual()
        self.command()
        self.now += 1.51
        with self.assertRaises(ValueError):
            self.command(sequence=2)
        self.assertEqual(self.supervisor.mode, 'prepared')
        self.assertEqual(self.hardware[0].writes[-1][1], 1500)

    def test_delayed_command_cannot_revive_previous_manual_generation(self):
        self.manual()
        old_generation = self.supervisor.generation
        self.command(sequence=1)
        self.now += 2
        self.supervisor.tick()
        self.action('enable', esc_on=True, stationary=True)
        with self.assertRaises(ValueError):
            self.action('command', generation=old_generation, sequence=999,
                        throttle_us=1570, steering_us=1700)
        self.supervisor.tick()
        self.assertEqual(self.hardware[0].writes[-1][1], 1500)

    def test_neutral_ack_allows_immediate_handoff_without_waiting_tick(self):
        self.manual()
        self.command()
        self.supervisor.tick()
        self.command(sequence=2, throttle=1500)
        self.start()
        self.assertEqual(self.supervisor.mode, 'autonomous')

    def test_forward_floor_and_reverse_rejected(self):
        self.manual()
        for value in (1000, 1400, 1501, 1569, 1601, 1570.0, True):
            with self.assertRaises(ValueError):
                self.command(throttle=value)

    def test_brake_pulse_survives_inflight_neutral_and_duplicate_brake(self):
        self.manual()
        self.action('brake')
        initial_deadline = self.supervisor.brake_until
        self.now += .2
        self.command(throttle=1500)
        self.action('brake')
        self.supervisor.tick()
        self.assertEqual(self.supervisor.brake_until, initial_deadline)
        self.assertEqual(self.hardware[0].writes[-1][1], 1400)
        self.now += .16
        self.supervisor.tick()
        self.assertEqual(self.hardware[0].writes[-1][1], 1500)

    def test_brake_never_resumes_queued_forward(self):
        self.manual()
        self.action('brake')
        self.command()
        self.now += .36
        self.supervisor.tick()
        self.assertEqual(self.hardware[0].writes[-1][1], 1500)

    def test_cannot_start_before_manual_neutral(self):
        self.manual()
        self.command()
        self.supervisor.tick()
        with self.assertRaises(ValueError):
            self.start()
        self.assertEqual(self.commands, [])

    def test_handoff_releases_uart_and_camera_before_cpp(self):
        self.manual()
        self.preview.start()
        self.start()
        self.assertEqual(self.supervisor.mode, 'autonomous')
        self.assertFalse(self.hardware[0].opened)
        self.assertIn('--handoff-complete', self.commands[0])
        self.assertIn('--supervised', self.commands[0])
        self.assertIn('--drive', self.commands[0])

    def advance_preparation(self, seconds):
        for _ in range(seconds * 4):
            self.now += .25
            self.supervisor.tick()

    def test_one_click_driving_test_can_be_repeated_from_idle(self):
        for task in ('parking', 'obstacle', 'traffic', 'zebra', 'race'):
            self.child = FakeProcess()
            response = self.action('start_test', task=task, handoff_complete=task == 'race')
            self.assertEqual((response['mode'], response['task']), ('test_preparing', task))
            before = len(self.commands)
            self.advance_preparation(2)
            self.assertEqual(len(self.commands), before)
            self.advance_preparation(1)
            self.assertEqual(len(self.commands), before + 1)
            self.assertIn(task, self.commands[-1])
            self.assertIn('--drive', self.commands[-1])
            self.assertEqual('--handoff-complete' in self.commands[-1], task == 'race')
            self.assertEqual(set(self.hardware[-1].writes), {(1610, 1500)})
            self.assertEqual(self.action('stop')['mode'], 'idle')
            self.assertTrue(self.child.terminated)
            self.assertTrue(all(not hardware.opened for hardware in self.hardware))

    def test_stop_cancels_pending_start_and_rejects_late_request(self):
        first = self.supervisor.generation
        self.action('start_test', task='parking')
        self.advance_preparation(2)
        with self.assertRaises(ValueError):
            self.action('start_test', task='zebra')
        self.supervisor.action('second-browser', {'action':'stop'})
        self.advance_preparation(5)
        with self.assertRaises(ValueError):
            self.action('start_test', generation=first, task='parking')
        self.assertEqual(self.commands, [])
        self.assertTrue(self.preview.running)
        self.assertFalse(self.hardware[0].opened)

    def test_test_setup_failure_releases_resources_and_cancels_launch(self):
        self.action('start_test', task='obstacle')
        self.hardware[0].fail = True
        self.advance_preparation(4)
        self.assertEqual(self.supervisor.mode, 'fault')
        self.assertIsNone(self.supervisor.owner)
        self.assertFalse(self.hardware[0].opened)
        self.assertTrue(self.preview.running)
        self.assertEqual(self.commands, [])

    def test_quick_observe_and_existing_control_ownership(self):
        with self.assertRaises(ValueError):
            self.action('start_test', task='race')
        with self.assertRaises(ValueError):
            self.start()
        self.assertEqual(self.action('start_test', task='observe')['mode'], 'observing')
        self.assertNotIn('--drive', self.commands[0])
        self.action('stop')
        self.assertEqual(self.hardware, [])
        self.manual()
        with self.assertRaises(PermissionError):
            self.supervisor.action('other', {'action':'start_test', 'task':'parking',
                                             'generation':self.supervisor.generation})
        with self.assertRaises(ValueError):
            self.action('start_test', task='parking')
        self.assertEqual(self.supervisor.mode, 'manual')

    def test_autonomous_does_not_depend_on_browser_heartbeat(self):
        self.manual()
        self.start()
        self.now += 600
        self.supervisor.tick()
        self.assertFalse(self.child.terminated)
        self.assertEqual(self.supervisor.mode, 'autonomous')
        with self.assertRaises(ValueError):
            self.command()

    def test_observe_never_opens_uart_even_on_stop(self):
        self.action('start', task='observe')
        self.assertNotIn('--drive', self.commands[0])
        self.action('stop')
        self.assertEqual(self.hardware, [])

    def test_camera_release_failure_does_not_launch_cpp(self):
        self.manual()
        self.preview.fail_stop = True
        with self.assertRaises(RuntimeError):
            self.start()
        self.assertEqual(self.commands, [])
        self.assertFalse(self.hardware[0].opened)
        self.assertEqual(self.supervisor.mode, 'fault')

    def test_child_exit_clears_old_forward_and_ownership(self):
        self.manual()
        self.start()
        self.child.returncode = 1
        self.supervisor.tick()
        self.assertEqual(self.supervisor.mode, 'fault')
        self.assertIsNone(self.supervisor.owner)
        self.assertTrue(all(t == 1500 for _, t in self.hardware[-1].writes))
        self.assertFalse(self.hardware[-1].opened)

    def test_failed_final_neutral_is_reported_unknown_not_stopped(self):
        self.manual()
        self.start()
        original_factory = self.supervisor.hardware_factory
        def failing_factory(port):
            hardware = original_factory(port)
            hardware.fail = True
            return hardware
        self.supervisor.hardware_factory = failing_factory
        self.child.returncode = 1
        with self.assertRaises(RuntimeError):
            self.supervisor.tick()
        status = self.supervisor.status()
        self.assertIsNone(status['throttle_us'])
        self.assertFalse(status['output_fresh'])
        self.assertEqual(status['output_source'], 'unknown_output_not_confirmed')

    def test_stop_terminates_child_then_neutral_and_release(self):
        self.manual()
        self.start()
        self.action('stop')
        self.assertTrue(self.child.terminated)
        self.assertEqual(self.supervisor.mode, 'idle')
        self.assertTrue(all(not item.opened for item in self.hardware))

    def test_stop_after_process_already_exited_is_idempotent(self):
        self.manual()
        self.start()
        self.child.returncode = 0
        self.action('stop')
        self.action('stop')
        self.assertEqual(self.supervisor.mode, 'idle')
        self.assertFalse(self.child.terminated)
        self.assertTrue(all(not item.opened for item in self.hardware))

    def test_autonomous_status_does_not_claim_unknown_output_neutral(self):
        self.manual()
        self.start()
        status = self.supervisor.status()
        self.assertIsNone(status['throttle_us'])
        self.assertFalse(status['output_fresh'])
        folder = self.supervisor.run_dir / '123'
        folder.mkdir()
        (folder / 'telemetry.jsonl').write_text('{"last_output_us":1570,"steering_us":1700}\n')
        status = self.supervisor.status()
        self.assertEqual(status['throttle_us'], 1570)
        self.assertEqual(status['steering_us'], 1700)
        self.assertTrue(status['output_fresh'])

    def test_slow_status_disk_read_does_not_block_stop(self):
        self.manual()
        self.start()
        entered, release, stopped = threading.Event(), threading.Event(), threading.Event()
        def slow_read(_):
            entered.set()
            release.wait(2)
            return None
        self.supervisor.telemetry = slow_read
        reader = threading.Thread(target=self.supervisor.status)
        reader.start()
        self.assertTrue(entered.wait(1))
        def stop():
            self.supervisor._stop()
            stopped.set()
        stopper = threading.Thread(target=lambda: self._under_lock(stop))
        stopper.start()
        try:
            self.assertTrue(stopped.wait(.5), 'disk I/O held the control lock')
        finally:
            release.set()
            reader.join(2)
            stopper.join(2)

    def _under_lock(self, operation):
        with self.supervisor.lock:
            operation()

    def test_manual_status_never_reads_previous_run_disk(self):
        self.manual()
        def forbidden_read(_):
            self.fail('manual status accessed old run disk')
        self.supervisor.telemetry = forbidden_read
        self.supervisor.status()

    def test_selected_task_is_whitelisted(self):
        with self.assertRaises(ValueError):
            self.start('../anything')

    def test_packet_matches_cpp_known_protocol(self):
        encoded = packet(1610, 1500)
        self.assertEqual(encoded[:8], bytes((0xaa, 0x55, 4, 10, 1, 20, 0, 2)))
        self.assertEqual(encoded[8:14], bytes((3, 0x4a, 6, 4, 0xdc, 5)))
        self.assertEqual(len(encoded), 15)

    def test_http_session_csrf_and_cross_origin(self):
        server = ControlServer(('127.0.0.1', 0), self.supervisor, self.root)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        connection = http.client.HTTPConnection('127.0.0.1', server.server_port)
        try:
            connection.request('GET', '/api/session')
            response = connection.getresponse()
            cookie = response.getheader('Set-Cookie').split(';', 1)[0]
            session = json.loads(response.read())
            self.assertNotEqual(cookie.split('=', 1)[1], session['session'])
            body = json.dumps({'action': 'prepare', 'esc_off': True, 'supervised': True})
            connection.request('POST', '/api/action', body, {'Cookie': cookie})
            response = connection.getresponse()
            self.assertEqual(response.status, 403)
            response.read()
            headers = {'Cookie': cookie, 'X-CSRF-Token': session['csrf'], 'Origin': 'http://evil.invalid'}
            connection.request('POST', '/api/action', body, headers)
            response = connection.getresponse()
            self.assertEqual(response.status, 403)
            response.read()
            headers.pop('Origin')
            connection.request('POST', '/api/action', body, headers)
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            self.assertEqual(json.loads(response.read())['mode'], 'prepared')
            connection.request('GET', '/api/status')
            response = connection.getresponse()
            status = json.loads(response.read())
            self.assertEqual(status['owner'], session['session'])
            self.assertNotIn(cookie.split('=', 1)[1], json.dumps(status))
            # The public owner label cannot authenticate a second browser.
            connection.request('GET', '/api/session', headers={'Cookie': 'smartcar_session=' + status['owner']})
            response = connection.getresponse()
            attacker_cookie = response.getheader('Set-Cookie').split(';', 1)[0]
            attacker_session = json.loads(response.read())
            self.assertNotEqual(attacker_session['session'], status['owner'])
            connection.request('POST', '/api/action', json.dumps({'action': 'enable', 'esc_on': True,
                               'stationary': True, 'generation': status['generation']}),
                               {'Cookie': attacker_cookie, 'X-CSRF-Token': attacker_session['csrf']})
            response = connection.getresponse()
            self.assertEqual(response.status, 403)
            response.read()
            # A valid new session may explicitly stop, but stopping never enables driving.
            connection.request('POST', '/api/action', json.dumps({'action': 'stop'}),
                               {'Cookie': attacker_cookie, 'X-CSRF-Token': attacker_session['csrf']})
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            self.assertEqual(json.loads(response.read())['mode'], 'idle')
            connection.request('GET', '/api/action?action=start&task=race')
            response = connection.getresponse()
            self.assertEqual(response.status, 404)
            response.read()
            self.assertEqual(self.commands, [])
        finally:
            connection.close()
            server.shutdown()
            server.server_close()
            thread.join(2)


if __name__ == '__main__':
    unittest.main()
