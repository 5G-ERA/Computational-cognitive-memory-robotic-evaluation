
from __future__ import annotations
from dataclasses import dataclass, asdict
from pathlib import Path
from collections import deque
import argparse, copy, json, math, statistics, time
REGIONS=['expected','adaptive','high_concern','dangerous']; RANK={r:i for i,r in enumerate(REGIONS)}
EXEC={'expected':1.0,'adaptive':0.8,'high_concern':0.4,'dangerous':0.0}
TAU={'expected:stable':0.0,'expected:towards_adaptive':0.2,'adaptive:towards_expected':0.2,'adaptive:stable':0.4,'adaptive:towards_high_concern':0.55,'high_concern:towards_adaptive':0.55,'high_concern:stable':0.70,'high_concern:towards_dangerous':0.85,'dangerous:towards_high_concern':0.85,'dangerous:stable_or_remains_dangerous':1.0}
EXP={'expected:stable':1.0,'expected:towards_adaptive':1.0,'adaptive:towards_expected':1.0,'adaptive:stable':1.0,'adaptive:towards_high_concern':0.5,'high_concern:towards_adaptive':0.5,'high_concern:stable':0.5,'high_concern:towards_dangerous':0.25,'dangerous:towards_high_concern':0.25,'dangerous:stable_or_remains_dangerous':0.25}
def load_json(p):
    with open(p,'r',encoding='utf-8') as f:return json.load(f)
def save_json(d,p):
    with open(p,'w',encoding='utf-8') as f:json.dump(d,f,indent=2,sort_keys=True);f.write('\n')
def clip01(x):
    try:x=float(x)
    except Exception:return 0.0
    return 0.0 if not math.isfinite(x) else max(0,min(1,x))
def norm(d):
    s=sum(max(0,float(v)) for v in d.values()); return {k:(max(0,float(v))/s if s else 1/len(d)) for k,v in d.items()}
def pos(d): return [k for k,v in d.items() if float(v)>0]
def neutral(d): return 1/max(1,len(pos(d)))
def classify(v,q):
    if not math.isfinite(float(v)):return 'dangerous'
    if v>q['expected']:return 'expected'
    if v>q['adaptive']:return 'adaptive'
    if v>q['dangerous']:return 'high_concern'
    return 'dangerous'
def direction(cur,prev=None,trend=None,belief=None,plaus=None):
    if trend is not None: imp,deg=trend>0,trend<0
    elif prev is not None: imp,deg=RANK[cur]<RANK[prev],RANK[cur]>RANK[prev]
    else:
        imp=deg=False
        if belief is not None and RANK[belief]>RANK[cur]:deg=True
        elif plaus is not None and RANK[plaus]<RANK[cur]:imp=True
    if cur=='expected': return 'towards_adaptive' if deg else 'stable'
    if cur=='adaptive': return 'towards_high_concern' if deg else ('towards_expected' if imp else 'stable')
    if cur=='high_concern': return 'towards_dangerous' if deg else ('towards_adaptive' if imp else 'stable')
    return 'towards_high_concern' if imp else 'stable_or_remains_dangerous'
def intent(tw,aw,flex,N):
    kt=(tw-N)/(1-N) if tw>=N else (N-tw)/N; dev=max(0,abs(tw-aw)-flex); return clip01(1-dev*kt),dev
@dataclass
class PS:
    meta_parameter:str; value:float; reliability:float; runtime_uncertainty:float; default_uncertainty:float; reliability_sensitivity:float; effective_margin:float
    belief_value:float; current_value:float; plausibility_value:float; belief_region:str; current_region:str; plausibility_region:str; tension_direction:str
    base_tension:float; attention_ratio:float; effective_attention_ratio:float; adjusted_tension:float; task_weight:float; analogy_weight:float; intent:float; deviation:float
    belief_fulfillment:float; current_fulfillment:float; plausibility_fulfillment:float; uncertainty_gap:float; stable_fulfillment:float; grounded:bool; dst_mode:str
@dataclass
class AS:
    analogy_id:str; local_analogy_tension:float; dominant_local_tension_meta_parameter:str|None; task_projected_tension:float
    task_belief_fulfillment:float; task_current_fulfillment:float; task_plausibility_fulfillment:float; task_uncertainty_gap:float; task_stable_fulfillment:float
    fulfillment_gate_status:str; tension_gate_passed:bool; fulfillment_gate_passed:bool; required_meta_gate_passed:bool; required_meta_failures:dict; uncertainty_gate_passed:bool
    hard_veto_triggered:bool; deployable:bool; rejection_reason:str|None; parameter_scores:dict
@dataclass
class Output:
    action:str; active_before:str; active_after:str; switch_to:str|None; reason:str; candidate_scores:dict; evaluation_controls:dict; timestamp:float
    def to_dict(self):
        return {'action':self.action,'active_before':self.active_before,'active_after':self.active_after,'switch_to':self.switch_to,'reason':self.reason,'timestamp':self.timestamp,'evaluation_controls':self.evaluation_controls,'candidate_scores':{a:{**{k:v for k,v in asdict(s).items() if k!='parameter_scores'},'parameter_scores':{m:asdict(p) for m,p in s.parameter_scores.items()}} for a,s in self.candidate_scores.items()}}
class MetaReasoner20:
    def __init__(self,cfg):
        self.config=load_json(cfg) if isinstance(cfg,(str,Path)) else copy.deepcopy(cfg); self.validate(); self.active=self.config.get('initial_active_analogy') or next(iter(self.analogies)); self.memory={}
    @property
    def analogies(self):return self.config['analogies']
    @property
    def task(self):return self.config['task_information']
    @property
    def eval(self):return self.config.get('evaluation_controls',{})
    def validate(self):
        for k in ['semantic_memory_regions','overall_meta_parameters','global_sensor_reliability','analogies','task_information']:
            if k not in self.config: raise ValueError('missing '+k)
        t=self.config['task_information']; t.setdefault('task_required_meta_thresholds',{}); t.setdefault('task_fulfillment_flexibility',{'enabled':False,'band':0,'borderline_policy':'fail'}); t.setdefault('task_dst',{'enabled':True,'uncertainty_penalty':0.8,'maximum_uncertainty_gap':0.3})
        self.config.setdefault('evaluation_controls',{'analogy_level_dst':{'enabled':True},'task_level_dst':{'enabled':True}})
    def sensor_defaults(self,mp,a):
        b=self.config['global_sensor_reliability'].get(mp,{}); o=a.get('sensor_reliability_override',{}).get(mp,{})
        return clip01(o.get('default_reliability',b.get('default_reliability',1.0))), float(o.get('reliability_sensitivity',b.get('reliability_sensitivity',0))), max(0,float(o.get('default_uncertainty',b.get('default_uncertainty',0))))
    def mem_params(self):
        rc=self.config.get('runtime_config',{}); ticks=rc.get('semantic_memory_window_ticks')
        if ticks is None: ticks=max(1,int(math.ceil(float(rc.get('frequency_hz',1))*float(rc.get('semantic_memory_window_seconds',5)))))
        return ticks,float(rc.get('direction_deadband',0.02))
    def trend_dir(self,aid,mp,val,cur,bel,pl):
        ticks,db=self.mem_params(); h=list(self.memory.get((aid,mp),[]))[-ticks:]
        if h:
            delta=val-statistics.median([x['value'] for x in h]); tr=1 if delta>db else (-1 if delta<-db else 0); return direction(cur,h[-1]['region'],tr,bel,pl)
        return direction(cur,belief=bel,plaus=pl)
    def push_mem(self,aid,mp,ts,val,reg):
        ticks,_=self.mem_params(); self.memory.setdefault((aid,mp),deque(maxlen=ticks)).append({'timestamp':ts,'value':val,'region':reg})
    def param(self,aid,mp,read,ts,tw,aw,maxatt,N):
        a=self.analogies[aid]; q=a.get('qoe',{}).get(mp)
        if q is None:return None
        rd=read.get(mp,float('nan')); rel0,sens,defu=self.sensor_defaults(mp,a)
        if isinstance(rd,dict): val=float(rd.get('value',float('nan'))); rel=clip01(rd.get('reliability',rel0)); runu=max(0,float(rd.get('uncertainty',defu)))
        else: val=float(rd); rel=rel0; runu=defu
        if self.eval.get('analogy_level_dst',{}).get('enabled',True):
            margin=runu+sens*(1-rel)+defu; bv,cv,pv=val-margin,val,val+margin; br,cr,pr=classify(bv,q),classify(cv,q),classify(pv,q); mode='analogy_dst_enabled'
        else:
            margin=0; bv=cv=pv=val; cr=classify(val,q); br=pr=cr; mode='analogy_dst_disabled_current_region_only'
        direc=self.trend_dir(aid,mp,val,cr,br,pr); base=self.config.get('analogy_tension_model',{}).get('region_direction_tension',TAU).get(f'{cr}:{direc}',TAU[f'{cr}:{direc}'])
        ratio=(aw/maxatt) if maxatt>0 and aw>0 else 0; exp=self.config.get('analogy_tension_model',{}).get('ratio_exponent_by_region_direction',EXP).get(f'{cr}:{direc}',EXP[f'{cr}:{direc}']); er=(ratio**exp) if ratio>0 else 0; adj=min(1,base*er)
        I,dev=intent(tw,aw,self.task.get('task_meta_attentions_flexibility',{}).get(mp,0),N) if tw>0 else (0,0)
        bf,cf,pf=I*EXEC[br],I*EXEC[cr],I*EXEC[pr]; gap=max(0,pf-bf); pen=float(self.task.get('task_dst',{}).get('uncertainty_penalty',0.8)); stable=clip01(cf*(1-pen*gap))
        self.push_mem(aid,mp,ts,val,cr)
        return PS(mp,val,rel,runu,defu,sens,margin,bv,cv,pv,br,cr,pr,direc,base,ratio,er,adj,tw,aw,I,dev,bf,cf,pf,gap,stable,True,mode)
    def score_analogy(self,aid,rin):
        ts=rin.get('timestamp',time.time()); read=rin.get('readings',rin); tws=norm(self.task['task_meta_attentions']); aws=norm(self.analogies[aid]['meta_attentions']); N=neutral(tws); maxatt=max([v for v in aws.values() if v>0] or [0])
        ps={}; local=[]; tb=[];tc=[];tp=[];g=[];tt=[]; hard=False; insuff=False
        for mp in set(tws)|set(aws):
            if tws.get(mp,0)<=0 and aws.get(mp,0)<=0: continue
            p=self.param(aid,mp,read,ts,tws.get(mp,0),aws.get(mp,0),maxatt,N)
            if p is None:
                if tws.get(mp,0)>self.task.get('missing_meta_parameter_policy',{}).get('negligible_task_weight',0.01): insuff=True
                continue
            ps[mp]=p
            if aws.get(mp,0)>0: local.append((mp,p.adjusted_tension))
            if tws.get(mp,0)>0: tb.append(p.belief_fulfillment); tc.append(p.current_fulfillment); tp.append(p.plausibility_fulfillment); g.append(p.uncertainty_gap); tt.append(p.base_tension)
            if p.current_region=='dangerous' and self.analogies[aid].get('hard_veto',{}).get(mp,False): hard=True
        belief,current,plaus=min(tb),min(tc),min(tp); ugap=max(g or [0]); tens=max(tt or [0])
        task_dst=self.eval.get('task_level_dst',{}).get('enabled',True) and self.task.get('task_dst',{}).get('enabled',True)
        if task_dst:
            maxgap=self.task['task_dst'].get('maximum_uncertainty_gap',None); ugate=True if maxgap is None else ugap<=float(maxgap); stable=clip01(current*(1-float(self.task['task_dst'].get('uncertainty_penalty',0.8))*ugap)); basis=belief
        else: ugap=0; ugate=True; stable=current; basis=current
        thr=float(self.task['task_fulfillment_threshold']); fcfg=self.task.get('task_fulfillment_flexibility',{}); band=float(fcfg.get('band',0)) if fcfg.get('enabled',False) else 0
        if basis>=thr: fpass=True; fstat='clear_pass'
        elif basis>=thr-band: fpass=fcfg.get('borderline_policy') in ('soft_pass','use_switch_persistence'); fstat='borderline'
        else: fpass=False; fstat='fail'
        tpass=tens<=float(self.task['task_tension_threshold']) and not hard
        reqfail={}
        for mp,rt in self.task.get('task_required_meta_thresholds',{}).items():
            val=(ps[mp].belief_fulfillment if task_dst else ps[mp].current_fulfillment) if mp in ps else 0
            if val<float(rt): reqfail[mp]={'fulfillment':val,'threshold':float(rt),'reason':'below_required_meta_threshold'}
        req=not reqfail; deploy=tpass and fpass and req and ugate and not insuff
        reason=None
        if hard: reason='hard_veto_dangerous'
        elif insuff: reason='insufficient_missing_grounding'
        elif not tpass: reason='tension_threshold_failed'
        elif not fpass: reason='fulfillment_threshold_failed'
        elif not req: reason='required_meta_threshold_failed'
        elif not ugate: reason='uncertainty_gap_too_large'
        return AS(aid,max([x[1] for x in local] or [0]),max(local,key=lambda x:x[1])[0] if local else None,tens,belief,current,plaus,ugap,stable,fstat,tpass,fpass,req,reqfail,ugate,hard,deploy,reason,ps)
    def score_all(self,rin): return {a:self.score_analogy(a,rin) for a in self.analogies}
    def decide(self,rin):
        ts=rin.get('timestamp',time.time()); before=self.active; scores=self.score_all(rin); dep={a:s for a,s in scores.items() if s.deployable}
        if any(s.hard_veto_triggered for s in scores.values()) and not dep: return Output('HELP',before,before,None,'Hard veto dangerous state detected.',scores,self.eval,ts)
        if not dep: return Output('FALLBACK',before,before,None,'No deployable analogy.',scores,self.eval,ts)
        cand=max(dep,key=lambda a:dep[a].task_stable_fulfillment); act='KEEP' if cand==before else 'SWITCH'
        if act=='SWITCH': self.active=cand
        return Output(act,before,self.active,None if act=='KEEP' else cand,'Decision completed.',scores,self.eval,ts)
    def calibration_report(self,calib):
        rows=[]; correct=0
        for c in calib.get('cases',[]):
            o=self.decide({'timestamp':c.get('timestamp',time.time()),'readings':c['reading']}).to_dict(); pred=o['switch_to'] or o['active_after'] if o['action'] in ('KEEP','SWITCH') else o['action']; exp=c['expected_output']; correct+=pred==exp
            rows.append({'case_id':c['case_id'],'expected':exp,'predicted':pred,'action':o['action'],'candidate_summary':{a:{'deployable':s['deployable'],'task_projected_tension':s['task_projected_tension'],'task_stable_fulfillment':s['task_stable_fulfillment'],'rejection_reason':s['rejection_reason']} for a,s in o['candidate_scores'].items()}})
        return {'version':'2.0','accuracy':correct/max(1,len(calib.get('cases',[]))),'evaluation_controls':self.eval,'case_results':rows}
def main(argv=None):
    p=argparse.ArgumentParser(); p.add_argument('--config',required=True); p.add_argument('--input'); p.add_argument('--output'); p.add_argument('--calibration'); p.add_argument('--calibration-report'); a=p.parse_args(argv); r=MetaReasoner20(a.config)
    if a.calibration:
        rep=r.calibration_report(load_json(a.calibration)); save_json(rep,a.calibration_report) if a.calibration_report else print(json.dumps(rep,indent=2))
    if a.input:
        out=r.decide(load_json(a.input)).to_dict(); save_json(out,a.output) if a.output else print(json.dumps(out,indent=2,sort_keys=True))
if __name__=='__main__': main()
