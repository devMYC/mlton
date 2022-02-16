/* Copyright (C) 2009,2012,2019,2021 Matthew Fluet.
 * Copyright (C) 1999-2008 Henry Cejtin, Matthew Fluet, Suresh
 *    Jagannathan, and Stephen Weeks.
 * Copyright (C) 1997-2000 NEC Research Institute.
 *
 * MLton is released under a HPND-style license.
 * See the file MLton-LICENSE for details.
 */

void displayGCState (GC_state s, FILE *stream) {
  fprintf (stream,
           "GC state\n");
  fprintf (stream, "\tcurrentThread = "FMTOBJPTR"\n", s->currentThread);
  displayThread (s, (GC_thread)(objptrToPointer (s->currentThread, s->heap.start)
                                + offsetofThread (s)), 
                 stream);
  fprintf (stream, "\tgenerational\n");
  displayGenerationalMaps (s, &s->generationalMaps, 
                           stream);
  fprintf (stream, "\theap\n");
  displayHeap (s, &s->heap, 
               stream);
  fprintf (stream,
           "\tlimit = "FMTPTR"\n"
           "\tstackBottom = "FMTPTR"\n"
           "\tstackTop = "FMTPTR"\n"
           "\tfrontier = "FMTPTR"\n",
           (uintptr_t)s->limit,
           (uintptr_t)s->stackBottom,
           (uintptr_t)s->stackTop,
           (uintptr_t)s->frontier);
}

size_t sizeofGCStateCurrentStackUsed (GC_state s) {
  return (size_t)(s->stackTop - s->stackBottom);
}

void setGCStateCurrentThreadAndStack (GC_state s) {
  GC_thread thread;
  GC_stack stack;

  thread = getThreadCurrent (s);
  s->exnStack = thread->exnStack;
  stack = getStackCurrent (s);
  s->stackBottom = getStackBottom (s, stack);
  s->stackTop = getStackTop (s, stack);
  s->stackLimit = getStackLimit (s, stack);
  markCard (s, (pointer)stack);
}

void setGCStateCurrentHeap (GC_state s, 
                            size_t oldGenBytesRequested,
                            size_t nurseryBytesRequested) {
  GC_heap h;
  pointer nursery;
  size_t nurserySize;
  pointer genNursery;
  size_t genNurserySize;

  if (DEBUG_DETAILED)
    fprintf (stderr, "setGCStateCurrentHeap(%s, %s)\n",
             uintmaxToCommaString(oldGenBytesRequested),
             uintmaxToCommaString(nurseryBytesRequested));
  h = &s->heap;
  assert (isFrontierAligned (s, h->start + h->oldGenSize + oldGenBytesRequested));
  s->limitPlusSlop = h->start + h->size;
  s->limit = s->limitPlusSlop - GC_HEAP_LIMIT_SLOP;
  nurserySize = h->size - (h->oldGenSize + oldGenBytesRequested);
  assert (isFrontierAligned (s, s->limitPlusSlop - nurserySize));
  nursery = s->limitPlusSlop - nurserySize;
  genNursery = alignFrontier (s, s->limitPlusSlop - (nurserySize / 2));
  genNurserySize = (size_t)(s->limitPlusSlop - genNursery);
  if (/* The mutator marks cards. */
      s->mutatorMarksCards
      /* There is enough space in the generational nursery. */
      and (nurseryBytesRequested <= genNurserySize)

      /* The nursery is large enough to be worth it. */
      and nurserySize >= 2 * NUR_SIZE_MIN

      and /* There is a reason to use generational GC. */
      (
       /* We must use it for debugging purposes. */
       FORCE_GENERATIONAL
       /* We just did a mark compact, so it will be advantageous to to use it. */
       or (s->lastMajorStatistics.kind == GC_MARK_COMPACT)
       /* The live ratio is low enough to make it worthwhile. */
       or ((float)h->size / (float)s->lastMajorStatistics.bytesLive
           <= (h->withMapsSize < s->sysvals.ram
               ? s->controls.ratios.copyGenerational
               : s->controls.ratios.markCompactGenerational))
       )) {
    s->canMinor = TRUE;
    if (nurseryBytesRequested <= h->fixedNurSize
        and h->fixedNurSize < genNurserySize)
    {
      nursery = alignFrontier (s, s->limitPlusSlop - h->fixedNurSize);
      nurserySize = h->fixedNurSize = (size_t)(s->limitPlusSlop - nursery);
      h->nurFixed = true;
      if (s->lastMajorStatistics.numMinorGCs == 0)
        h->nurThresh = NUR_THRESH_MUL
                     * (size_t)(nursery - (h->start + h->oldGenSize + oldGenBytesRequested))
                     / nurserySize;
    } else {
      nursery = genNursery;
      nurserySize = genNurserySize;
      h->nurFixed = false;
    }
    clearCardMap (s);
  } else {
    unless (nurseryBytesRequested <= nurserySize)
      die ("Out of memory.  Insufficient space in nursery.");
    s->canMinor = FALSE;
  }
  assert (nurseryBytesRequested <= nurserySize);
  s->heap.nursery = nursery;
  s->frontier = nursery;
  assert (nurseryBytesRequested <= (size_t)(s->limitPlusSlop - s->frontier));
  assert (isFrontierAligned (s, s->heap.nursery));
  assert (hasHeapBytesFree (s, oldGenBytesRequested, nurseryBytesRequested));
  if (s->controls.messages)
    fprintf (stderr, "-------- canMinor=%d\n", s->canMinor);
}

Bool_t GC_getAmOriginal (GC_state s) {
  return (Bool_t)(s->amOriginal);
}
void GC_setAmOriginal (GC_state s, Bool_t b) {
  s->amOriginal = (bool)b;
}

void GC_setControlsMessages (GC_state s, Bool_t b) {
  s->controls.messages = (bool)b;
}

void GC_setControlsSummary (GC_state s, Bool_t b) {
  s->controls.summary = (bool)b;
}

void GC_setControlsRusageMeasureGC (GC_state s, Bool_t b) {
  s->controls.rusageMeasureGC = (bool)b;
}

uintmax_t GC_getCumulativeStatisticsBytesAllocated (GC_state s) {
  return s->cumulativeStatistics.bytesAllocated;
}

uintmax_t GC_getCumulativeStatisticsNumCopyingGCs (GC_state s) {
  return s->cumulativeStatistics.numCopyingGCs;
}

uintmax_t GC_getCumulativeStatisticsNumMarkCompactGCs (GC_state s) {
  return s->cumulativeStatistics.numMarkCompactGCs;
}

uintmax_t GC_getCumulativeStatisticsNumMinorGCs (GC_state s) {
  return s->cumulativeStatistics.numMinorGCs;
}

size_t GC_getCumulativeStatisticsMaxBytesLive (GC_state s) {
  return s->cumulativeStatistics.maxBytesLive;
}

void GC_setHashConsDuringGC (GC_state s, Bool_t b) {
  s->hashConsDuringGC = (bool)b;
}

size_t GC_getLastMajorStatisticsBytesLive (GC_state s) {
  return s->lastMajorStatistics.bytesLive;
}


pointer GC_getCallFromCHandlerThread (GC_state s) {
  pointer p = objptrToPointer (s->callFromCHandlerThread, s->heap.start);
  return p;
}

void GC_setCallFromCHandlerThread (GC_state s, pointer p) {
  objptr op = pointerToObjptr (p, s->heap.start);
  s->callFromCHandlerThread = op;
}

pointer GC_getCallFromCOpArgsResPtr (GC_state s) {
  return s->callFromCOpArgsResPtr;
}

pointer GC_getCurrentThread (GC_state s) {
  pointer p = objptrToPointer (s->currentThread, s->heap.start);
  return p;
}

pointer GC_getSavedThread (GC_state s) {
  pointer p;

  assert(s->savedThread != BOGUS_OBJPTR);
  p = objptrToPointer (s->savedThread, s->heap.start);
  s->savedThread = BOGUS_OBJPTR;
  return p;
}

void GC_setSavedThread (GC_state s, pointer p) {
  objptr op;

  assert(s->savedThread == BOGUS_OBJPTR);
  op = pointerToObjptr (p, s->heap.start);
  s->savedThread = op;
}

void GC_setSignalHandlerThread (GC_state s, pointer p) {
  objptr op = pointerToObjptr (p, s->heap.start);
  s->signalHandlerThread = op;
}

struct rusage* GC_getRusageGCAddr (GC_state s) {
  return &(s->cumulativeStatistics.ru_gc);
}

sigset_t* GC_getSignalsHandledAddr (GC_state s) {
  return &(s->signalsInfo.signalsHandled);
}

sigset_t* GC_getSignalsPendingAddr (GC_state s) {
  return &(s->signalsInfo.signalsPending);
}

void GC_setGCSignalHandled (GC_state s, Bool_t b) {
  s->signalsInfo.gcSignalHandled = (bool)b;
}

Bool_t GC_getGCSignalPending (GC_state s) {
  return (bool)(s->signalsInfo.gcSignalPending);
}

void GC_setGCSignalPending (GC_state s, Bool_t b) {
  s->signalsInfo.gcSignalPending = (bool)b;
}
