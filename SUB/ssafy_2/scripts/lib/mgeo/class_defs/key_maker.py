#!/usr/bin/env python
# -*- coding: utf-8 -*-

class KeyMaker(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self, prefix):
        self.prefix = prefix
        self.num = -1
    
    def get_new(self):
        self.num += 1
        if self.prefix == '':
            # prefix가 0이면, 그냥 숫자로 반환
            return self.num
        else:
            return '{}{:06d}'.format(self.prefix, self.num)