#!/usr/bin/env python3

import time

class Timer:
    def __init__(self):
        self.time = time.time()

    def print_time(self):
        print(self.time)

    def update_time(self):
        self.time = time.time()

    def measure_time(self):
        current_time = time.time()
        elapsed_time = current_time - self.time
        return elapsed_time
