"""Conservative exposure-normalized red evidence, independent per camera.

Unknown startup frames are never an off reference. A reference requires three
fresh, same-fixture low-exposure OFF observations. Saturated references cannot
recover clipped information. No inverse brightness threshold is used.
"""
from .traffic_tracking import same_fixture


class RedPhotometry:
    def __init__(self):
        self.reference=None
        self.pending=None
        self.last_id=self.last_stamp=None

    def update(self, signal, fid, captured, now):
        result=dict(signal)
        fs=signal.get('lamp_features',[])
        box=signal.get('bbox_xyxy');housing=signal.get('housing_median_v')
        ordered=(type(fid) is int and isinstance(captured,(float,int)) and
                 (self.last_id is None or fid>self.last_id and captured>self.last_stamp))
        if not ordered or not 0<=now-captured<=.25:
            return result
        gap=None if self.last_stamp is None else captured-self.last_stamp
        self.last_id,self.last_stamp=fid,captured
        if len(fs)!=3 or box is None or housing is None or housing<15:
            self.pending=None
            return result
        red=fs[0]
        if self.reference is not None and not same_fixture(self.reference['box'],box):
            self.reference=self.pending=None
        # Never learn sunlight-clipped plastic, ambiguous frames, or a dim
        # coloured lamp just because its median is below the old 230 threshold.
        eligible=(signal['state']=='off' and max(f['median_v'] for f in fs)<160 and
                  all(f['clipped_fraction']<.1 and f['white_fraction']<.02 for f in fs))
        if eligible:
            count=(self.pending['count']+1 if self.pending and gap is not None and
                   gap<=.25+1e-9 and same_fixture(self.pending['box'],box) else 1)
            self.pending=dict(box=box,count=count)
            if count>=3:
                self.reference=dict(box=box,red=dict(red),housing=housing,stamp=captured)
        else:self.pending=None
        ref=self.reference
        result['red_photometry']={'reference_available':ref is not None,'reason':'no_qualified_off_reference'}
        if ref is None:return result
        # Old scene/position references expire; changing to a nearby fixture
        # cannot inherit their off reference or green permission.
        if captured-ref['stamp']>25:
            self.reference=None
            result['red_photometry']['reason']='off_reference_expired'
            return result
        gain=housing/ref['housing']
        compensated=red['median_v']/gain-ref['red']['median_v'] if gain>0 else 0
        peak_rise=red['led_peak_fraction']-ref['red']['led_peak_fraction']
        usable=(.5<=gain<=2 and ref['red']['clipped_fraction']<.1 and red['clipped_fraction']<.4)
        # Both local light production and a change in the LED structure are
        # required; global dimming alone never means powered red.
        emission=(usable and compensated>=30 and red['local_contrast']>=15 and
                  red['led_peak_fraction']>=.06 and peak_rise>=.03 and
                  (red['color_fraction']>=.35 or red['white_fraction']>=.1 and red['distributed_halo']))
        result['red_photometry'].update(exposure_gain=gain,compensated_red_rise=compensated,
            led_peak_rise=peak_rise,usable=usable,reason='red_emission_change' if emission else 'no_verified_red_emission_change')
        if emission:
            result.update(state='red',signal_for_race='red',reason='exposure_normalized_red_emission')
        return result
