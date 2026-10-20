"""Ограниченная очередь задержек; timestamp источника не меняется."""
import heapq
import math
import random

class DelayChannel:
    def __init__(self, delay=0., jitter=0., loss=0., capacity=1000, seed=42):
        if delay < 0 or jitter < 0 or not 0 <= loss <= 1 or capacity <= 0:
            raise ValueError('Некорректные параметры канала')
        self.delay, self.jitter, self.loss, self.capacity = delay, jitter, loss, capacity
        self.random = random.Random(seed)
        self.queue = []
        self.sequence = self.dropped = 0
        self.blocked_until = None
        self.previous_time = None

    def observe_time(self, now):
        if not math.isfinite(now):
            raise ValueError('Некорректное время канала')
        if self.previous_time is not None and now < self.previous_time:
            self.queue.clear()
            self.blocked_until = None
        self.previous_time = now
        if self.blocked_until is not None and now >= self.blocked_until:
            self.blocked_until = None

    def drop_for(self, now, duration):
        if not math.isfinite(duration) or duration <= 0 or not math.isfinite(now + duration):
            raise ValueError('Некорректная длительность потери')
        self.observe_time(now)
        self.dropped += len(self.queue)
        self.queue.clear()
        self.blocked_until = now + duration

    def send(self, message, now):
        self.observe_time(now)
        if self.blocked_until is not None:
            self.dropped += 1
            return False
        if self.random.random() < self.loss or len(self.queue) >= self.capacity:
            self.dropped += 1
            return False
        self.sequence += 1
        due = now + max(0., self.delay + self.random.uniform(-self.jitter, self.jitter))
        heapq.heappush(self.queue, (due, self.sequence, message))
        return True

    def receive(self, now):
        self.observe_time(now)
        messages = []
        while self.queue and self.queue[0][0] <= now:
            messages.append(heapq.heappop(self.queue)[2])
        return messages

