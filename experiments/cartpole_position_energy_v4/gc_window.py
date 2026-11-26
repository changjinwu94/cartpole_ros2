"""Scoped cyclic-GC control for finite commissioning trials; no ROS dependency."""
import gc
import time

class GcWindow:
    def __init__(self, collector=gc, clock=time.monotonic_ns):
        self.gc=collector;self.clock=clock;self.original=collector.isenabled()
        self.phase='startup';self.events=[];self.starts={};self.begun=False;self.closed=False
        self.disabled_at=None;self.restored_at=None
        self.callback=self._callback
        collector.callbacks.append(self.callback)
    def _callback(self,phase,info):
        generation=info.get('generation',-1);now=self.clock()
        if phase=='start':self.starts[generation]=(now,self.phase)
        elif phase=='stop':
            start,where=self.starts.pop(generation,(now,self.phase))
            self.events.append(dict(generation=generation,phase=where,start_ns=start,end_ns=now,
                duration_ms=(now-start)/1e6,collected=info.get('collected'),uncollectable=info.get('uncollectable')))
    def begin(self):
        if self.begun or self.closed:raise RuntimeError('GC window already started or closed')
        self.begun=True
        self.gc.disable()
        self.phase='pretrial_collect_drive_disabled'
        # Caller verifies disabled before this explicit collection.
        self.gc.collect()
        self.phase='bounded_trial'
        self.disabled_at=self.clock()
    def close(self):
        if self.closed:return
        self.phase='restore_after_callback_shutdown'
        try:
            if self.original:self.gc.enable()
            else:self.gc.disable()
            self.restored_at=self.clock()
        finally:
            if self.callback in self.gc.callbacks:self.gc.callbacks.remove(self.callback)
            self.closed=True
    def report(self):
        return dict(originally_enabled=self.original,window_started=self.begun,
            disabled_at_ns=self.disabled_at,restored_at_ns=self.restored_at,
            restored=self.closed and self.gc.isenabled()==self.original,
            enabled_now=self.gc.isenabled(),collections=list(self.events),
            collections_during_trial=sum(e['phase']=='bounded_trial' for e in self.events))
