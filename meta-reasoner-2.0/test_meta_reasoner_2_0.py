
import unittest, json, copy
from meta_reasoner_2_0 import MetaReasoner20, TAU, EXP
class TestMR20(unittest.TestCase):
    def cfg(self): return json.load(open('config_meta_reasoner_2_0.json'))
    def test_compile_threshold(self):
        self.assertLessEqual(TAU['adaptive:stable'],0.50); self.assertGreater(TAU['adaptive:towards_high_concern'],0.50)
    def test_open_keep(self):
        r=MetaReasoner20(self.cfg()); out=r.decide(json.load(open('sample_inputs/open_corridor.json'))); self.assertEqual(out.action,'KEEP')
    def test_unsafe_help(self):
        r=MetaReasoner20(self.cfg()); out=r.decide(json.load(open('sample_inputs/unsafe.json'))); self.assertEqual(out.action,'HELP')
    def test_search_lid_rejected(self):
        r=MetaReasoner20(self.cfg()); out=r.decide(json.load(open('sample_inputs/open_corridor.json'))); self.assertFalse(out.candidate_scores['Search_Lid'].deployable)
    def test_threshold_050_rejects_worsening_cautious_after_memory(self):
        r=MetaReasoner20(self.cfg()); r.decide(json.load(open('sample_inputs/open_corridor.json'))); out=r.decide(json.load(open('sample_inputs/narrow_door.json'))); self.assertFalse(out.candidate_scores['Cautious_Nav'].tension_gate_passed)
    def test_zero_attention_battery_ignored_in_task(self):
        r=MetaReasoner20(self.cfg()); out=r.decide(json.load(open('sample_inputs/open_corridor.json'))); self.assertNotIn('battery_consumption', out.candidate_scores['Efficient_Nav'].parameter_scores)
    def test_battery_task_required(self):
        c=json.load(open('config_meta_reasoner_2_0_battery.json')); r=MetaReasoner20(c); out=r.decide({'timestamp':9,'readings':{'progression':1.2,'safety':1.2,'fragility':1.0,'battery_consumption':0.1}}); self.assertFalse(out.candidate_scores['Efficient_Nav_Low_Battery'].tension_gate_passed)
    def test_disable_analogy_dst(self):
        c=self.cfg(); c['evaluation_controls']['analogy_level_dst']['enabled']=False; r=MetaReasoner20(c); out=r.decide(json.load(open('sample_inputs/narrow_door.json'))); ps=out.candidate_scores['Efficient_Nav'].parameter_scores['safety']; self.assertEqual(ps.belief_region,ps.current_region); self.assertEqual(ps.plausibility_region,ps.current_region)
    def test_disable_task_dst(self):
        c=self.cfg(); c['evaluation_controls']['task_level_dst']['enabled']=False; r=MetaReasoner20(c); out=r.decide(json.load(open('sample_inputs/narrow_door.json'))); s=out.candidate_scores['Efficient_Nav']; self.assertEqual(s.task_uncertainty_gap,0); self.assertAlmostEqual(s.task_stable_fulfillment,s.task_current_fulfillment)
    def test_exponents(self):
        self.assertEqual(EXP['high_concern:stable'],0.5); self.assertEqual(EXP['high_concern:towards_dangerous'],0.25)
    def test_required_meta_general(self):
        c=self.cfg(); c['task_information']['task_required_meta_thresholds']={'safety':0.99}; r=MetaReasoner20(c); out=r.decide(json.load(open('sample_inputs/narrow_door.json'))); self.assertTrue(any(not s.required_meta_gate_passed for s in out.candidate_scores.values()))
if __name__=='__main__': unittest.main(verbosity=2)
