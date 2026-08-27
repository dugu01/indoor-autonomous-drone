#!/usr/bin/env python3
from s24py.core import Config
from s24py.policy import adversarial_policy_contract
from s24py.scenario2 import Geometry,run_trial
from s24py.search_parallel import search_parallel
p=adversarial_policy_contract();print('CONTROLLED POLICY:',p['pass'])
b=run_trial(Geometry(),Config());print('PHYSICAL BASELINE:',b.pass_contract,b.reasons,b.metrics)
nom,w=search_parallel(200,7,Config(),4);print('nominal:',len(nom),'robust:',len(w))
if w:print('best:',w[0].geometry,w[0].metrics)
