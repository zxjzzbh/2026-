"""Saving validated cone settings must not invalidate the software review."""
import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from carvision.boot_test import reviewed_manifest
from carvision.cone_observer import ConeObserverDrive


CONFIG = 'vision/configs/cone-trial.local.json'
ROOT = Path(__file__).resolve().parents[2]


def reviewed_workspace(tmp_path):
    settings = json.loads((ROOT/'vision/configs/cone-trial.json').read_text(encoding='utf-8'))
    path = tmp_path/CONFIG
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps(settings), encoding='utf-8')
    (tmp_path/'code.py').write_text('checked executable', encoding='utf-8')
    manifest = {'pi_checks': {'rc': 0}, 'files': [
        {'path': rel, 'sha256': hashlib.sha256((tmp_path/rel).read_bytes()).hexdigest()}
        for rel in (CONFIG, 'code.py')]}
    (tmp_path/'checks.json').write_text(json.dumps(manifest), encoding='utf-8')
    return settings, path, {'software_checks_record': 'checks.json'}, manifest


@pytest.mark.parametrize('white,forward', [(200,1600),(210,1610),(255,1575)])
def test_real_ui_save_remains_reviewed_without_mutating_manifest(tmp_path, white, forward):
    settings,path,template,manifest = reviewed_workspace(tmp_path)
    candidate = copy.deepcopy(settings)
    candidate.setdefault('scene',{})['white_v_min'] = white
    candidate.setdefault('driving',{})['forward_us'] = forward
    observer = SimpleNamespace(local_config=path,settings=settings)
    ConeObserverDrive._save(observer,candidate)
    assert hashlib.sha256(path.read_bytes()).hexdigest() != manifest['files'][0]['sha256']
    before = (tmp_path/'checks.json').read_bytes()
    assert reviewed_manifest(tmp_path,template) == manifest
    assert (tmp_path/'checks.json').read_bytes() == before
    assert json.loads(path.read_text(encoding='utf-8')) == candidate


@pytest.mark.parametrize('section,key,value', [
    ('scene','white_v_min',256),('driving','forward_us',1700),
    ('driving','actuator_lease_s',5),('driving','odometry_verified','true')])
def test_invalid_editable_values_are_still_rejected(tmp_path, section, key, value):
    settings,path,template,_ = reviewed_workspace(tmp_path)
    settings.setdefault(section,{})[key] = value
    path.write_text(json.dumps(settings),encoding='utf-8')
    with pytest.raises(ValueError,match='避障调试参数无效'):
        reviewed_manifest(tmp_path,template)


def test_source_drift_still_rejected_after_parameter_save(tmp_path):
    settings,path,template,_ = reviewed_workspace(tmp_path)
    ConeObserverDrive._save(SimpleNamespace(local_config=path,settings=settings),settings)
    (tmp_path/'code.py').write_text('unreviewed change',encoding='utf-8')
    with pytest.raises(ValueError,match='code.py'):
        reviewed_manifest(tmp_path,template)


def test_other_local_files_keep_hash_checks(tmp_path):
    _,_,template,manifest = reviewed_workspace(tmp_path)
    other = 'vision/configs/other.local.json'
    path = tmp_path/other
    path.write_text('{}',encoding='utf-8')
    manifest['files'].append({'path':other,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    (tmp_path/'checks.json').write_text(json.dumps(manifest),encoding='utf-8')
    path.write_text('{"changed":true}',encoding='utf-8')
    with pytest.raises(ValueError,match='other.local.json'):
        reviewed_manifest(tmp_path,template)


@pytest.mark.parametrize('content', ['{broken', 'null', '[]'])
def test_malformed_editable_file_reports_parameter_error(tmp_path, content):
    _,path,template,_ = reviewed_workspace(tmp_path)
    path.write_text(content,encoding='utf-8')
    with pytest.raises(ValueError,match='避障调试参数无效'):
        reviewed_manifest(tmp_path,template)
