import json
import threading
import urllib.request
import urllib.error
from pathlib import Path

import cv2
import numpy as np
import pytest

from carvision.traffic_capture import TrafficCollector
from carvision.traffic_capture_server import RemoteFeed, make_server


def jpeg(value=30):
    ok,data=cv2.imencode('.jpg',np.full((120,160,3),value,np.uint8));assert ok
    return data.tobytes()


@pytest.fixture
def store(tmp_path):
    now=[0.0]
    c=TrafficCollector(tmp_path,clock=lambda:now[0])
    return c,now


def test_frozen_sample_saves_exact_displayed_jpeg_not_a_new_live_frame(store):
    c,now=store;old=jpeg(30);frame=c.publish('primary',old,{})
    token=c.freeze({'primary':frame})
    now[0]=.5;c.publish('primary',jpeg(80),{})
    row=c.capture(token,'primary','red',[.1,.2,.7,.8],{'distance_cm':100})
    assert (c.folder/row['image']).read_bytes()==old
    assert row['bbox_xyxy']==[16,24,112,96]
    assert row['label']=='red' and row['prediction']['state']=='unknown'
    assert not row['prediction']['is_ground_truth'] and row['metadata']['distance_cm']==100


def test_prediction_never_becomes_annotation_and_originals_are_not_overwritten(store):
    c,_=store;f=c.publish('primary',jpeg(),{})
    token=c.freeze({'primary':f});row=c.capture(token,'primary','unlabeled')
    assert row['annotation_status']=='unreviewed'
    with pytest.raises(ValueError):c.capture(token,'primary','green')
    c.relabel(row['sample_id'],'no_light')
    saved=json.loads((c.folder/'labels'/(row['sample_id']+'.json')).read_text(encoding='utf-8'))
    assert saved['label']=='no_light' and (c.folder/row['image']).read_bytes()==jpeg()
    assert Path(c.export()).is_file()


@pytest.mark.parametrize('box',[[0,0,2,1],[.5,.2,.1,.3],[0,0,float('nan'),1]])
def test_invalid_boxes_are_rejected_before_saving(store,box):
    c,_=store;f=c.publish('primary',jpeg(),{});token=c.freeze({'primary':f})
    with pytest.raises(ValueError):c.capture(token,'primary','green',box)
    assert not c.items


def test_no_light_negative_cannot_silently_keep_a_positive_box(store):
    c,_=store;f=c.publish('primary',jpeg(),{});token=c.freeze({'primary':f})
    with pytest.raises(ValueError):c.capture(token,'primary','no_light',[.1,.1,.5,.5])


def test_new_capture_rejects_stale_feed_but_frozen_annotation_remains_stable(store):
    c,now=store;f=c.publish('primary',jpeg(),{});token=c.freeze({'primary':f})
    now[0]=4
    with pytest.raises(ValueError):c.freeze({'primary':f})
    assert c.capture(token,'primary','off')['label']=='off'
    now[0]=301
    with pytest.raises(ValueError):c.capture(token,'primary','off')


def test_sequence_keeps_timing_even_for_identical_pixels_and_does_not_assign_red(store):
    c,now=store;c.start_recording(['primary','secondary'],seconds=2,fps=2,meta={'scene':'transition'})
    for t in (0,.1,.5,1,1.5):
        now[0]=t
        for cam in ('primary','secondary'):c.publish(cam,jpeg(),{})
    assert len(c.items)==8
    assert all(r['label']=='unlabeled' and r['sequence_id'] for r in c.items.values())
    assert any(r['same_pixels_seen_before'] for r in c.items.values())
    with pytest.raises(ValueError):c.new_session()
    now[0]=2.1;status=c.status()
    assert status['recording'] is None and status['last_recording']['saved']==8
    assert status['last_recording']['reason']=='duration_reached'


def test_camera_names_and_scene_names_never_become_arbitrary_paths(store):
    c,_=store
    with pytest.raises(ValueError):c.publish('../escape',jpeg(),{})
    f=c.publish('primary',jpeg(),{});token=c.freeze({'primary':f})
    row=c.capture(token,'primary','red',meta={'scene':'../../do-not-use-as-path'})
    assert (c.folder/row['image']).resolve().is_relative_to(c.root)
    with pytest.raises(ValueError):c.frame_bytes('../escape')


def test_feed_has_only_two_get_paths_and_does_not_claim_exact_remote_timestamp(store):
    c,_=store;feed=RemoteFeed('http://example.invalid:8080',c);paths=[]
    def get(opener,path,limit):
        paths.append(path)
        if path.startswith('/api/status'):return json.dumps({'raw_preview':{'state':'ok','host_frame_age_ms':20,'frame_id':10}}).encode()
        return jpeg()
    feed.get=get;f=feed.fetch_once('primary',None)
    assert paths==['/api/status?camera=primary','/frame.jpg?camera=primary&view=raw']
    assert c.cache[f]['source']['status_is_exact_jpeg_timestamp'] is False
    assert not any('drive' in p for p in paths)


def test_local_http_rejects_untrusted_origin_and_never_exposes_drive_actions(store):
    c,_=store
    asset=Path(__file__).parents[1]/'web/traffic_capture.html'
    server=make_server(0,c,asset)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    base='http://127.0.0.1:'+str(server.server_port)
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        assert json.loads(opener.open(base+'/api/status').read())['hardware_output'] is False
        req=urllib.request.Request(base+'/api/session',data=b'{}',headers={'Content-Type':'application/json','Origin':'http://untrusted.invalid'})
        with pytest.raises(urllib.error.HTTPError) as e:opener.open(req)
        assert e.value.code==403
        with pytest.raises(urllib.error.HTTPError) as e:opener.open(base+'/api/drive/command')
        assert e.value.code==404
    finally:server.shutdown();server.server_close();thread.join(1)


def populate(c,now,count=76):
    c.start_recording(['primary','secondary'],seconds=60,fps=5)
    for i in range(count):
        now[0]=i*.25
        c.publish('primary' if i%2==0 else 'secondary',jpeg(i+10),{})
    c.stop_recording()
    return list(c.items)


def test_all_76_samples_are_reachable_by_pages_and_early_pages_do_not_shift(store):
    c,now=store;ids=populate(c,now)
    pages=[c.samples(page=i,page_size=16) for i in range(1,6)]
    assert [len(p['items']) for p in pages]==[16,16,16,16,12]
    assert [r['sample_id'] for p in pages for r in p['items']]==ids
    assert pages[-1]['first']==65 and pages[-1]['last']==76
    assert c.samples(page=100)['page']==5
    now[0]+=1;c.publish('primary',jpeg(200),{})
    token=c.freeze({'primary':c.latest['primary']['id']});c.capture(token,'primary','red')
    assert [r['sample_id'] for r in c.samples()['items']]==ids[:16]


def test_filtering_unreviewed_and_camera_keeps_precise_totals(store):
    c,now=store;ids=populate(c,now,20)
    c.relabel(ids[0],'red');c.relabel(ids[1],'green')
    page=c.samples(label='unlabeled',camera='primary',page_size=16)
    assert page['session_total']==20 and page['total']==9
    assert all(r['camera']=='primary' and r['label']=='unlabeled' for r in page['items'])
    assert page['counts']['red']==1 and page['counts']['green']==1


def test_equal_wallclock_times_keep_capture_order_before_and_after_restart(store,monkeypatch):
    c,now=store
    monkeypatch.setattr('carvision.traffic_capture.utc_now',lambda:'2026-10-09T12:00:00+00:00')
    ids=populate(c,now,20)
    c.relabel(ids[0],'green')
    assert [r['sample_id'] for r in c.samples(page_size=32)['items']]==ids
    restored=TrafficCollector(c.root)
    assert [r['sample_id'] for r in restored.samples(page_size=32)['items']]==ids


def test_restart_resumes_active_batch_and_history_edit_does_not_change_capture_batch(store):
    c,now=store;ids=populate(c,now,20);old=c.session_id
    c.relabel(ids[0],'green')
    image_before=c.sample_bytes(old,ids[0])
    restored=TrafficCollector(c.root)
    assert restored.session_id==old and len(restored.items)==20
    assert restored.items[ids[0]]['label']=='green'
    new=restored.new_session()
    assert new!=old and not restored.items
    assert len(restored.samples(old,page=2)['items'])==4
    restored.relabel(ids[0],'red',old)
    assert restored.session_id==new and not restored.items
    assert restored.sample_bytes(old,ids[0])==image_before
    label=json.loads((restored.root/old/'labels'/(ids[0]+'.json')).read_text(encoding='utf-8'))
    assert label['label']=='red'
    assert Path(restored.export(old)).is_file()
    assert TrafficCollector(c.root).session_id==new


@pytest.mark.parametrize('session',['../outside','/absolute','20261009T140315Z-00000000'])
def test_review_rejects_arbitrary_or_missing_batch_paths(store,session):
    c,_=store
    with pytest.raises(ValueError):c.samples(session)
    with pytest.raises(ValueError):c.sample_bytes(session,'a'*32)


def test_history_metadata_cannot_redirect_image_reads_outside_batch(store):
    c,now=store;ids=populate(c,now,1);old=c.session_id
    c.new_session();path=c.root/old/'labels'/(ids[0]+'.json')
    row=json.loads(path.read_text(encoding='utf-8'));row['image']='../../private.jpg'
    path.write_text(json.dumps(row),encoding='utf-8')
    with pytest.raises(ValueError):c.samples(old)
    with pytest.raises(ValueError):c.sample_bytes(old,ids[0])


def test_http_page_five_serves_older_images_and_all_batches(store):
    c,now=store;ids=populate(c,now);old=c.session_id;c.new_session()
    server=make_server(0,c,Path(__file__).parents[1]/'web/traffic_capture.html')
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    base='http://127.0.0.1:'+str(server.server_port)
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        sessions=json.loads(opener.open(base+'/api/sessions').read())
        assert len(sessions['sessions'])==2
        page=json.loads(opener.open(base+'/api/samples?session='+old+'&page=5&page_size=16').read())
        assert page['total']==76 and len(page['items'])==12
        assert opener.open(base+'/sample/'+ids[-1]+'.jpg?session='+old).read()==c.sample_bytes(old,ids[-1])
        assert c.session_id!=old
    finally:server.shutdown();server.server_close();thread.join(1)
