"""Export existing experiment summaries and one matched simulator trace for the site.

Does not retrain models. Run with the objectpilot venv after diagnostic runs exist.
"""
import json
import csv
import shutil
from pathlib import Path
from objectpilot.core import PID
from objectpilot.env import ApproachEnv
from objectpilot.evaluate import load_policy

root=Path(__file__).resolve().parents[1]
out=root/'docs/evidence'
out.mkdir(parents=True,exist_ok=True)
result={'date':'2026-10-29','evidence_type':'synthetic sensor simulation','episodes':30,'seed_start':10000,'controllers':[]}
for key,title,folder in [('pid','PID starting gains','pid_initial'),('ppo_smoke','PPO 8,192 steps','ppo_smoke_eval'),('ppo','PPO 100,352 steps','ppo_initial_eval')]:
    summary=json.loads((root/'runs'/folder/'summary.json').read_text())
    rows=list(csv.DictReader((root/'runs'/folder/'episodes.csv').open()))
    successes=sum(row['success']=='True' for row in rows)
    assert successes/len(rows)==summary['success_rate']
    summary['success_95pct_wilson']=[max(0,v) for v in summary['success_95pct_wilson']]
    result['controllers'].append({'id':key,'title':title,'successes':successes,**summary})
    shutil.copyfile(root/'runs'/folder/'episodes.csv',out/f'{key}-episodes.csv')
(out/'results.json').write_text(json.dumps(result,indent=2),encoding='utf-8')

trace={'seed':10000,'dt':.1,'stand_off_m':.65,'minimum_m':.4,'note':'Recorded Python simulator rollout, not flight footage. First diagnostic seed, not selected for success.','controllers':{}}
model=load_policy(root/'runs/ppo_initial/policy.zip')
for name in ('pid','ppo'):
    env=ApproachEnv(); obs,info=env.reset(seed=10000); pid=PID()
    samples=[dict(t=0,x=env.x,y=env.y,heading=env.heading,range_m=None,forward=0,yaw=0,reason='initial',outcome='running')]
    while True:
        action=pid.predict(obs) if name=='pid' else model.predict(obs,deterministic=True)[0]
        obs,_,term,trunc,info=env.step(action)
        f=obs[-16:]
        samples.append(dict(t=round(info['time_s'],2),x=round(env.x,4),y=round(env.y,4),heading=round(env.heading,4),
                            range_m=round(float(f[4]*1.2),3) if f[5]>.5 else None,
                            forward=round(info['safe_action'][0],3),yaw=round(info['safe_action'][1],3),
                            reason=info['supervisor'],outcome=info['outcome']))
        if term or trunc: break
    trace['controllers'][name]={'samples':samples,'outcome':info['outcome']}
    env.close()
(out/'trajectory.json').write_text(json.dumps(trace,separators=(',',':')),encoding='utf-8')
print(json.dumps({'results':[(r['title'],r['successes']) for r in result['controllers']],
                  'trace':{k:v['outcome'] for k,v in trace['controllers'].items()}}))
