"""Ограниченная очередь задержек; timestamp источника не меняется."""
import heapq
import random

class DelayChannel:
    def __init__(self, delay=0., jitter=0., loss=0., capacity=1000, seed=42):
        if delay < 0 or jitter < 0 or not 0 <= loss <= 1 or capacity <= 0:
            raise ValueError('Некорректные параметры канала')
        self.delay, self.jitter, self.loss, self.capacity = delay, jitter, loss, capacity
        self.random = random.Random(seed)
        self.queue = []
        self.sequence = self.dropped = 0

    def send(self, message, now):
        if self.random.random() < self.loss or len(self.queue) >= self.capacity:
            self.dropped += 1
            return False
        self.sequence += 1
        due = now + max(0., self.delay + self.random.uniform(-self.jitter, self.jitter))
        heapq.heappush(self.queue, (due, self.sequence, message))
        return True

    def receive(self, now):
        messages = []
        while self.queue and self.queue[0][0] <= now:
            messages.append(heapq.heappop(self.queue)[2])
        return messages

