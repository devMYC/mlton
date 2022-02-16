/* Copyright (C) 2009-2010,2012,2016 Matthew Fluet.
 * Copyright (C) 1999-2008 Henry Cejtin, Matthew Fluet, Suresh
 *    Jagannathan, and Stephen Weeks.
 * Copyright (C) 1997-2000 NEC Research Institute.
 *
 * MLton is released under a HPND-style license.
 * See the file MLton-LICENSE for details.
 */

void minorGC (GC_state s) {
  minorCheneyCopyGC (s);
}

void majorGC (GC_state s, size_t bytesRequested, bool mayResize) {
  uintmax_t numGCs;
  size_t desiredSize;
  uintmax_t numMinorGCs;
  int nurInc;

  numMinorGCs = s->lastMajorStatistics.numMinorGCs;
  s->lastMajorStatistics.numMinorGCs = 0;
  const size_t heapSizeBefore = s->heap.size;
  const size_t lastBytesLive = s->lastMajorStatistics.bytesLive;
  numGCs =
    s->cumulativeStatistics.numCopyingGCs 
    + s->cumulativeStatistics.numMarkCompactGCs;
  if (0 < numGCs
      and ((float)(s->cumulativeStatistics.numHashConsGCs) / (float)(numGCs)
           < s->controls.ratios.hashCons))
    s->hashConsDuringGC = TRUE;

  nurInc = 0;
  if (s->canMinor and s->heap.nurFixed) {
    if (numMinorGCs < 0.9 * s->heap.nurThresh)
      nurInc = (int)((float)(s->heap.nurThresh - numMinorGCs) / s->heap.nurThresh * s->heap.fixedNurSize);
    else if (numMinorGCs > 1.1 * s->heap.nurThresh)
      nurInc = -(int)((float)(numMinorGCs - s->heap.nurThresh) / s->heap.nurThresh * s->heap.fixedNurSize);
    if (nurInc < 0 and s->heap.fixedNurSize - (size_t)(-nurInc) < NUR_SIZE_MIN)
      nurInc = -1 * (s->heap.fixedNurSize - NUR_SIZE_MIN);
    desiredSize = (size_t)(s->heap.size * (1 + s->winStatistics.movAvgMajorLiveRatio)) + (size_t)max(nurInc, 0);
    desiredSize = min (desiredSize, s->sysvals.ram);
  } else {
    desiredSize =
      sizeofHeapDesired (s, s->lastMajorStatistics.bytesLive + bytesRequested, 0);
  }
  if (not FORCE_MARK_COMPACT
      and not s->hashConsDuringGC // only markCompact can hash cons
      and s->heap.withMapsSize < s->sysvals.ram
      and (not isHeapInit (&s->secondaryHeap)
           or createHeapSecondary (s, desiredSize)))
    majorCheneyCopyGC (s);
  else
    majorMarkCompactGC (s);
  s->hashConsDuringGC = FALSE;
  s->lastMajorStatistics.bytesLive = s->heap.oldGenSize;

  float liveRatio = (float)s->lastMajorStatistics.bytesLive / heapSizeBefore;
  s->cumulativeStatistics.avgMajorLiveRatio =
    (s->cumulativeStatistics.avgMajorLiveRatio * numGCs + liveRatio) / (numGCs+1);
  float* win = s->winStatistics.recentMajorLiveRatios;
  if (win[MOV_AVG_WIN_SIZE-1] >= 0.0f) {
    s->winStatistics.movAvgMajorLiveRatio = (s->winStatistics.movAvgMajorLiveRatio
                                             * MOV_AVG_WIN_SIZE
                                             - win[0]
                                             + liveRatio)
                                            / MOV_AVG_WIN_SIZE;
    memmove(win, win+1, (MOV_AVG_WIN_SIZE-1) * sizeof(float));
    win[MOV_AVG_WIN_SIZE-1] = liveRatio;
  } else {
    float sum = liveRatio;
    unsigned int i = 0;
    for (; win[i] >= 0.0f; i++)
      sum += win[i];
    win[i] = liveRatio;
    s->winStatistics.movAvgMajorLiveRatio = sum / (i+1);
  }
  if (s->controls.messages) {
    fprintf (stderr, "[GC: "
                     "Live/Heap=%s/%s (%.2f%%), "
                     "AvgLR=%.2f%%, "
                     "MovAvgLR=%.2f%%]\n",
             uintmaxToCommaString(s->lastMajorStatistics.bytesLive),
             uintmaxToCommaString(heapSizeBefore),
             100*liveRatio,
             100*s->cumulativeStatistics.avgMajorLiveRatio,
             100*s->winStatistics.movAvgMajorLiveRatio);
    fprintf (stderr, "-------- recentLR=[");
    for (int i = 0; i < MOV_AVG_WIN_SIZE; i++)
        fprintf (stderr, "%.2f%s", 100*win[i], i+1==MOV_AVG_WIN_SIZE ? "" : ", ");
    fprintf (stderr, "]\n");
    fprintf (stderr, "[GC: LiveDataGrowth=%.2f%%, AllocRateGrowth=%.2f%%]\n",
             100*((double)s->lastMajorStatistics.bytesLive - lastBytesLive) / lastBytesLive,
             100*((double)s->winStatistics.movAllocRate - s->cumulativeStatistics.avgAllocRate) / s->cumulativeStatistics.avgAllocRate);
  }

  if (s->lastMajorStatistics.bytesLive > s->cumulativeStatistics.maxBytesLive)
    s->cumulativeStatistics.maxBytesLive = s->lastMajorStatistics.bytesLive;

  if (s->canMinor and s->heap.nurFixed and s->heap.shrinkFlag) {
    s->heap.shrinkFlag = false;
    const size_t keepSize = s->heap.size
                          - (size_t)((s->heap.nursery
                                      - max (nurInc, 0)
                                      - (s->heap.start + s->lastMajorStatistics.bytesLive))
                                     * min (0.5, (float)(numMinorGCs - s->heap.nurThresh) / numMinorGCs));
    shrinkHeap (s, &s->heap, keepSize);
  }

  /* Notice that the s->lastMajorStatistics.bytesLive below is
   * different than the s->lastMajorStatistics.bytesLive used as an
   * argument to createHeapSecondary above.  Above, it was an
   * estimate.  Here, it is exactly how much was live after the GC.
   */
  else if (mayResize) {
    resizeHeap (s, s->lastMajorStatistics.bytesLive + bytesRequested);
  }
  setCardMapAndCrossMap (s);
  resizeHeapSecondary (s);
  assert (s->heap.oldGenSize + bytesRequested <= s->heap.size);
  if (s->canMinor and s->heap.nurFixed and not (nurInc == 0)) {
    if (nurInc < 0) s->heap.fixedNurSize -= (size_t)(-nurInc);
    else s->heap.fixedNurSize += (size_t)(nurInc);
  }
}

void growStackCurrent (GC_state s) {
  size_t reserved;
  GC_stack stack;

  reserved = sizeofStackGrowReserved (s, getStackCurrent(s));
  if (DEBUG_STACKS or s->controls.messages)
    fprintf (stderr, 
             "[GC: Growing stack of size %s bytes to size %s bytes, using %s bytes.]\n",
             uintmaxToCommaString(getStackCurrent(s)->reserved),
             uintmaxToCommaString(reserved),
             uintmaxToCommaString(getStackCurrent(s)->used));
  assert (hasHeapBytesFree (s, sizeofStackWithMetaData (s, reserved), 0));
  stack = newStack (s, reserved, TRUE);
  copyStack (s, getStackCurrent(s), stack);
  getThreadCurrent(s)->stack = pointerToObjptr ((pointer)stack, s->heap.start);
  markCard (s, objptrToPointer (getThreadCurrentObjptr(s), s->heap.start));
}

void enterGC (GC_state s) {
  if (s->profiling.isOn) {
    /* We don't need to profileEnter for count profiling because it
     * has already bumped the counter.  If we did allow the bump, then
     * the count would look like function(s) had run an extra time.
     */  
    if (s->profiling.stack
        and not (PROFILE_COUNT == s->profiling.kind))
      GC_profileEnter (s);
  }
  s->amInGC = TRUE;
}

void leaveGC (GC_state s) {
  if (s->profiling.isOn) {
    if (s->profiling.stack
        and not (PROFILE_COUNT == s->profiling.kind))
      GC_profileLeave (s);
  }
  s->amInGC = FALSE;
}

void performGC (GC_state s, 
                size_t oldGenBytesRequested,
                size_t nurseryBytesRequested, 
                bool forceMajor,
                bool mayResize) {
  uintmax_t gcTime;
  bool stackTopOk;
  size_t stackBytesRequested;
  struct rusage ru_start;
  size_t totalBytesRequested;

  enterGC (s);
  s->cumulativeStatistics.numGCs++;
  if (DEBUG or s->controls.messages) {
    size_t nurserySize = s->heap.size - ((size_t)(s->heap.nursery - s->heap.start));
    size_t nurseryUsed = (size_t)(s->frontier - s->heap.nursery);
    fprintf (stderr, 
             "[GC: Starting gc #%s; requesting %s nursery bytes and %s old-gen bytes,]\n",
             uintmaxToCommaString(s->cumulativeStatistics.numGCs),
             uintmaxToCommaString(nurseryBytesRequested),
             uintmaxToCommaString(oldGenBytesRequested));
    fprintf (stderr, 
             "[GC:\theap at "FMTPTR" of size %s bytes (+ %s bytes card/cross map),]\n",
             (uintptr_t)(s->heap.start),
             uintmaxToCommaString(s->heap.size),
             uintmaxToCommaString(s->heap.withMapsSize - s->heap.size));
    fprintf (stderr, 
             "[GC:\twith old-gen of size %s bytes (%.1f%% of heap),]\n",
             uintmaxToCommaString(s->heap.oldGenSize),
             100.0 * ((double)(s->heap.oldGenSize) / (double)(s->heap.size)));
    fprintf (stderr,
             "[GC:\tand nursery of size %s bytes (%.1f%% of heap),]\n",
             uintmaxToCommaString(nurserySize),
             100.0 * ((double)(nurserySize) / (double)(s->heap.size)));
    fprintf (stderr,
             "[GC:\tand nursery using %s bytes (%.1f%% of heap, %.1f%% of nursery).]\n",
             uintmaxToCommaString(nurseryUsed),
             100.0 * ((double)(nurseryUsed) / (double)(s->heap.size)),
             100.0 * ((double)(nurseryUsed) / (double)(nurserySize)));
  }
  assert (invariantForGC (s));
  if (needGCTime (s))
    startTiming (&ru_start);
  minorGC (s);
  stackTopOk = invariantForMutatorStack (s);
  stackBytesRequested = 
    stackTopOk 
    ? 0 
    : sizeofStackWithMetaData (s, sizeofStackGrowReserved (s, getStackCurrent (s)));
  totalBytesRequested = 
    oldGenBytesRequested 
    + nurseryBytesRequested
    + stackBytesRequested;
  if (forceMajor 
      or totalBytesRequested > s->heap.size - s->heap.oldGenSize)
    majorGC (s, totalBytesRequested, mayResize);
  setGCStateCurrentHeap (s, oldGenBytesRequested + stackBytesRequested, 
                         nurseryBytesRequested);
  assert (hasHeapBytesFree (s, oldGenBytesRequested + stackBytesRequested,
                            nurseryBytesRequested));
  unless (stackTopOk)
    growStackCurrent (s);
  setGCStateCurrentThreadAndStack (s);
  if (needGCTime (s)) {
    gcTime = stopTiming (&ru_start, &s->cumulativeStatistics.ru_gc);
    s->cumulativeStatistics.maxPauseTime = 
      max (s->cumulativeStatistics.maxPauseTime, gcTime);
  } else
    gcTime = 0;  /* Assign gcTime to quell gcc warning. */
  if (DEBUG or s->controls.messages) {
    size_t nurserySize = s->heap.size - (size_t)(s->heap.nursery - s->heap.start);
    fprintf (stderr, 
             "[GC: Finished gc #%s; time %s ms,]\n",
             uintmaxToCommaString(s->cumulativeStatistics.numGCs),
             uintmaxToCommaString(gcTime));
    fprintf (stderr, 
             "[GC:\theap at "FMTPTR" of size %s bytes (+ %s bytes card/cross map),]\n",
             (uintptr_t)(s->heap.start),
             uintmaxToCommaString(s->heap.size),
             uintmaxToCommaString(s->heap.withMapsSize - s->heap.size));
    fprintf (stderr, 
             "[GC:\twith old-gen of size %s bytes (%.1f%% of heap),]\n",
             uintmaxToCommaString(s->heap.oldGenSize),
             100.0 * ((double)(s->heap.oldGenSize) / (double)(s->heap.size)));
    fprintf (stderr,
             "[GC:\tand nursery of size %s bytes (%.1f%% of heap).]\n",
             uintmaxToCommaString(nurserySize),
             100.0 * ((double)(nurserySize) / (double)(s->heap.size)));
  }
  /* Send a GC signal. */
  if (s->signalsInfo.gcSignalHandled
      and s->signalHandlerThread != BOGUS_OBJPTR) {
    if (DEBUG_SIGNALS)
      fprintf (stderr, "GC Signal pending.\n");
    s->signalsInfo.gcSignalPending = TRUE;
    unless (s->signalsInfo.amInSignalHandler) 
      s->signalsInfo.signalIsPending = TRUE;
  }
  if (DEBUG) 
    displayGCState (s, stderr);
  assert (hasHeapBytesFree (s, oldGenBytesRequested, nurseryBytesRequested));
  assert (invariantForGC (s));
  leaveGC (s);
  gettimeofday (&s->lastMajorStatistics.prevDoneAt, 0);
}

void ensureInvariantForMutator (GC_state s, bool force) {
  if (force
      or not (invariantForMutatorFrontier(s))
      or not (invariantForMutatorStack(s))) {
    /* This GC will grow the stack, if necessary. */
    performGC (s, 0, getThreadCurrent(s)->bytesNeeded, force, TRUE);
  }
  assert (invariantForMutatorFrontier(s));
  assert (invariantForMutatorStack(s));
}

/* ensureHasHeapBytesFree (s, oldGen, nursery) 
 */
void ensureHasHeapBytesFree (GC_state s, 
                             size_t oldGenBytesRequested,
                             size_t nurseryBytesRequested) {
  assert (s->heap.nursery <= s->limitPlusSlop);
  assert (s->frontier <= s->limitPlusSlop);
  if (not hasHeapBytesFree (s, oldGenBytesRequested, nurseryBytesRequested))
    performGC (s, oldGenBytesRequested, nurseryBytesRequested, FALSE, TRUE);
  assert (hasHeapBytesFree (s, oldGenBytesRequested, nurseryBytesRequested));
}

void GC_collect (GC_state s, size_t bytesRequested, bool force) {
  enter (s);
  /* When the mutator requests zero bytes, it may actually need as
   * much as GC_HEAP_LIMIT_SLOP.
   */
  if (0 == bytesRequested)
    bytesRequested = GC_HEAP_LIMIT_SLOP;
  getThreadCurrent(s)->bytesNeeded = bytesRequested;
  switchToSignalHandlerThreadIfNonAtomicAndSignalPending (s);
  ensureInvariantForMutator (s, force);
  assert (invariantForMutatorFrontier(s));
  assert (invariantForMutatorStack(s));
  leave (s);
}
