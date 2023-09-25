#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)
sys.path.append(os.path.normpath(os.path.join(current_path, '../'))) 


class Version:
    def __init__(self, maj_ver, min_ver):
        self.maj = int(maj_ver)
        self.min = int(min_ver)

    def __lt__(self, other):
        if self.maj < other.maj:
            return True
        elif self.maj > other.maj:
            return False
        else:
            return self.min < other.min
    
    def __le__(self, other):
        if self.maj < other.maj:
            return True
        elif self.maj > other.maj:
            return False
        else:
            return self.min <= other.min
    
    def __gt__(self, other):
        if self.maj > other.maj:
            return True
        elif self.maj < other.maj:
            return False
        else:
            return self.min > other.min
    
    def __ge__(self, other):
        if self.maj > other.maj:
            return True
        elif self.maj < other.maj:
            return False
        else:
            return self.min >= other.min
    
    def __eq__(self, other):
        return (self.maj == other.maj) and (self.min == other.min)


