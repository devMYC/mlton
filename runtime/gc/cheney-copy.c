/* Copyright (C) 2012,2016,2019 Matthew Fluet.
 * Copyright (C) 1999-2008 Henry Cejtin, Matthew Fluet, Suresh
 *    Jagannathan, and Stephen Weeks.
 * Copyright (C) 1997-2000 NEC Research Institute.
 *
 * MLton is released under a HPND-style license.
 * See the file MLton-LICENSE for details.
 */

/* ---------------------------------------------------------------- */
/*                    Cheney Copying Collection                     */
/* ---------------------------------------------------------------- */

void updateWeaksForCheneyCopy (GC_state s) {
  pointer p;
  GC_weak w;

  for (w = s->weaks; w != NULL; w = w->link) {
    assert (BOGUS_OBJPTR != w->objptr);

    if (DEBUG_WEAK)
      fprintf (stderr, "updateWeaksForCheneyCopy  w = "FMTPTR"  ", (uintptr_t)w);
    p = objptrToPointer (w->objptr, s->heap.start);
    if (hasFwdPtr(p)) {
      if (DEBUG_WEAK)
        fprintf (stderr, "forwarded from "FMTOBJPTR" to "FMTOBJPTR"\n",
                 w->objptr, getFwdPtr(p));
      w->objptr = getFwdPtr(p);
    } else {
      if (DEBUG_WEAK)
        fprintf (stderr, "cleared\n");
      *(getHeaderp((pointer)w - offsetofWeak (s))) = GC_WEAK_GONE_HEADER;
      w->objptr = BOGUS_OBJPTR;
    }
  }
  s->weaks = NULL;
}

void swapHeapsForCheneyCopy (GC_state s) {
  struct GC_heap tempHeap;

  tempHeap = s->secondaryHeap;
  s->secondaryHeap = s->heap;
  s->heap = tempHeap;
  setCardMapAndCrossMap (s);
}

void majorCheneyCopyGC (GC_state s) {
  size_t bytesCopied;
  struct GC_forwardState forwardState;
  struct GC_foreachObjptrClosure forwardObjptrIfInHeapClosure;
  struct rusage ru_start;
  pointer toStart;

  assert (s->secondaryHeap.size >= s->heap.oldGenSize);
  if (detailedGCTime (s))
    startTiming (&ru_start);
  s->cumulativeStatistics.numCopyingGCs++;
  if (DEBUG or s->controls.messages) {
    fprintf (stderr, 
             "[GC: Starting major Cheney-copy;]\n");
    fprintf (stderr,
             "[GC:\tfrom heap at "FMTPTR" of size %s bytes,]\n",
             (uintptr_t)(s->heap.start), 
             uintmaxToCommaString(s->heap.size));
    fprintf (stderr, 
             "[GC:\tto heap at "FMTPTR" of size %s bytes.]\n",
             (uintptr_t)(s->secondaryHeap.start), 
             uintmaxToCommaString(s->secondaryHeap.size));
  }
  assert (s->secondaryHeap.start != (pointer)NULL);
  /* The next assert ensures there is enough space for the copy to
   * succeed.  It does not assert 
   *   (s->secondaryHeap.size >= s->heap.size) 
   * because that is too strong.
   */
  assert (s->secondaryHeap.size >= s->heap.oldGenSize);
  toStart = alignFrontier (s, s->secondaryHeap.start);
  forwardState.amInMinorGC = FALSE;
  forwardState.toStart = s->secondaryHeap.start;
  forwardState.toLimit = s->secondaryHeap.start + s->secondaryHeap.size;
  forwardState.back = toStart;
  forwardObjptrIfInHeapClosure.fun = forwardObjptrIfInHeapFun;
  forwardObjptrIfInHeapClosure.env = &forwardState;
  foreachGlobalObjptr (s, &forwardObjptrIfInHeapClosure);
  foreachObjptrInRange (s, toStart, &forwardState.back,
                        &forwardObjptrIfInHeapClosure, TRUE);
  updateWeaksForCheneyCopy (s);
  s->secondaryHeap.oldGenSize = (size_t)(forwardState.back - s->secondaryHeap.start);
  bytesCopied = s->secondaryHeap.oldGenSize;
  s->cumulativeStatistics.bytesCopied += bytesCopied;
  swapHeapsForCheneyCopy (s);
  s->lastMajorStatistics.kind = GC_COPYING;
  if (detailedGCTime (s))
    stopTiming (&ru_start, &s->cumulativeStatistics.ru_gcCopying);
  if (DEBUG or s->controls.messages)
    fprintf (stderr, 
             "[GC: Finished major Cheney-copy; copied %s bytes.]\n",
             uintmaxToCommaString(bytesCopied));
}

/* ---------------------------------------------------------------- */
/*                 Minor Cheney Copying Collection                  */
/* ---------------------------------------------------------------- */

void minorCheneyCopyGC (GC_state s) {
  size_t bytesAllocated;
  size_t bytesCopied;
  struct GC_forwardState forwardState;
  struct GC_foreachObjptrClosure forwardObjptrIfInNurseryClosure;
  struct rusage ru_start;

  if (DEBUG_GENERATIONAL)
    fprintf (stderr, "minorGC  nursery = "FMTPTR"  frontier = "FMTPTR"\n",
             (uintptr_t)s->heap.nursery, (uintptr_t)s->frontier);
  assert (invariantForGC (s));
  bytesAllocated = (size_t)(s->frontier - s->heap.nursery);
  if (bytesAllocated == 0)
    return;
  s->cumulativeStatistics.bytesAllocated += bytesAllocated;
  if (not s->canMinor) {
    s->heap.oldGenSize += bytesAllocated;
  } else {
    if (detailedGCTime (s))
      startTiming (&ru_start);
    s->cumulativeStatistics.numMinorGCs++;
    if (DEBUG_GENERATIONAL or s->controls.messages) {
      fprintf (stderr, 
               "[GC: Starting minor Cheney-copy;]\n");
      fprintf (stderr,
               "[GC:\tfrom nursery at "FMTPTR" of size %s bytes.]\n",
               (uintptr_t)(s->heap.nursery),
               uintmaxToCommaString(bytesAllocated));
    }
    assert (invariantForGC (s));
    forwardState.amInMinorGC = TRUE;
    forwardState.toStart = s->heap.start + s->heap.oldGenSize;
    forwardState.toLimit = forwardState.toStart + bytesAllocated;
    forwardState.back = forwardState.toStart;
    forwardObjptrIfInNurseryClosure.fun = forwardObjptrIfInNurseryFun;
    forwardObjptrIfInNurseryClosure.env = &forwardState;
    assert (isFrontierAligned (s, forwardState.toStart));
    /* Forward all globals.  Would like to avoid doing this once all
     * the globals have been assigned.
     */
    foreachGlobalObjptr (s, &forwardObjptrIfInNurseryClosure);
    forwardInterGenerationalObjptrs (s, &forwardState);
    foreachObjptrInRange (s, forwardState.toStart, &forwardState.back,
                          &forwardObjptrIfInNurseryClosure, TRUE);
    updateWeaksForCheneyCopy (s);
    bytesCopied = (size_t)(forwardState.back - forwardState.toStart);
    s->cumulativeStatistics.bytesCopiedMinor += bytesCopied;
    s->heap.oldGenSize += bytesCopied;
    s->lastMajorStatistics.numMinorGCs++;
    if (detailedGCTime (s))
      stopTiming (&ru_start, &s->cumulativeStatistics.ru_gcMinor);
    if (DEBUG_GENERATIONAL or s->controls.messages)
      fprintf (stderr, 
               "[GC: Finished minor Cheney-copy; copied %s bytes.]\n",
               uintmaxToCommaString(bytesCopied));
    if (s->heap.nurFixed
        and not s->heap.shrinkFlag
        and s->lastMajorStatistics.numMinorGCs > s->heap.nurThresh
        and (s->heap.nursery - (s->heap.start + s->heap.oldGenSize))
              / (float)(s->heap.nursery - (s->heap.start + s->lastMajorStatistics.bytesLive))
            > SHRINK_THRESH)
      s->heap.shrinkFlag = true;

  }

  const long million = 1000000;
  const uintmax_t prevMovAllocRate = s->winStatistics.movAllocRate;
  struct timeval end;
  gettimeofday (&end, 0);
  struct rusage total;
  getrusage (RUSAGE_SELF, &total);

  struct rusage *ru_gc = &s->cumulativeStatistics.ru_gc;
  const long gcusec = (ru_gc->ru_utime.tv_sec + ru_gc->ru_stime.tv_sec)
                    * million
                    + ru_gc->ru_utime.tv_usec
                    + ru_gc->ru_stime.tv_usec;

  const long totalusec = (total.ru_utime.tv_sec + total.ru_stime.tv_sec)
                       * million
                       + total.ru_utime.tv_usec
                       + total.ru_stime.tv_usec;

  const long nonGCusec = totalusec - gcusec;

  struct timeval *start = &s->lastMajorStatistics.prevDoneAt;
  const long usecFromStart = (end.tv_sec - start->tv_sec)
                           * million
                           + end.tv_usec
                           - start->tv_usec;

  const uintmax_t avgAllocRate = (uintmax_t)((double)s->cumulativeStatistics.bytesAllocated / nonGCusec * million);
  s->cumulativeStatistics.avgAllocRate = avgAllocRate;

  const uintmax_t allocRate = (uintmax_t)((double)bytesAllocated / usecFromStart * million);
  struct GC_winStatistics *win = &s->winStatistics;

  if (win->recentMinorAllocRates[MOV_AVG_WIN_SIZE-1] >= 0) {
      win->movAllocRate = (win->movAllocRate * MOV_AVG_WIN_SIZE
                          - (uintmax_t)win->recentMinorAllocRates[0]
                          + allocRate
                          ) / MOV_AVG_WIN_SIZE;
      memmove (win->recentMinorAllocRates,
               win->recentMinorAllocRates+1,
               (MOV_AVG_WIN_SIZE-1) * sizeof(long long));
      win->recentMinorAllocRates[MOV_AVG_WIN_SIZE-1] = (long long)allocRate;
  } else {
      unsigned int i = 0;
      uintmax_t sum = allocRate;
      for (; i < MOV_AVG_WIN_SIZE && win->recentMinorAllocRates[i] >= 0; i++)
        sum += (uintmax_t)win->recentMinorAllocRates[i];
      win->recentMinorAllocRates[i] = (long long)allocRate;
      win->movAllocRate = sum / (i+1);
  }

  if (s->controls.messages) {
    fprintf (stderr, "[GC: Allocated/Elapsed: "
                     "%s bytes/%s usecs "
                     "(%s bytes/sec).]\n",
             uintmaxToCommaString(bytesAllocated),
             uintmaxToCommaString(usecFromStart),
             uintmaxToCommaString(allocRate));
    fprintf (stderr, "[GC: Moving allocation rate %s bytes/sec (%s%.2f%%).]\n",
             uintmaxToCommaString(win->movAllocRate),
             win->movAllocRate >= prevMovAllocRate ? "" : "-",
             prevMovAllocRate == 0
               ? 0.0f
               : 100*(float)(win->movAllocRate >= prevMovAllocRate
                   ? win->movAllocRate - prevMovAllocRate
                   : prevMovAllocRate - win->movAllocRate) / prevMovAllocRate);
    fprintf (stderr, "-------- [");
    for (int i = 0; i < MOV_AVG_WIN_SIZE; i++)
        fprintf (stderr, "%s%s",
                 win->recentMinorAllocRates[i] < 0
                   ? "-1"
                   : uintmaxToCommaString((uintmax_t)win->recentMinorAllocRates[i]),
                 i+1 >= MOV_AVG_WIN_SIZE ? "" : ", ");
    fprintf (stderr, "]\n");
    fprintf (stderr, "[GC: Average allocation rate %s bytes/sec.]\n",
             uintmaxToCommaString(avgAllocRate));
  }
}
