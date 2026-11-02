"""Bound repeated forward/backward oscillation without advancing on prediction."""
from collections import deque

class OscillationMonitor:
    def __init__(self):
        self.index=None;self.started=0;self.history=deque();self.cooldown_until=0
    def reset(self):
        self.index=None;self.history.clear()
    def observe(self,step,index):
        if index!=self.index:
            self.index=index;self.started=step;self.history.clear()
        while self.history and self.history[0][0]<step-24:self.history.popleft()
        signs=[sign for _,sign in self.history if sign]
        reversals=sum(a!=b for a,b in zip(signs,signs[1:]))
        return step>=self.cooldown_until and step-self.started>=18 and reversals>=4
    def record(self,step,action):
        sign=1 if int(action)&1 else -1 if int(action)&2 else 0
        self.history.append((step,sign))
    def recovered(self,step):
        self.reset();self.cooldown_until=step+48
