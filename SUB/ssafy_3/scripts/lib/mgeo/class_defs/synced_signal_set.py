#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from class_defs.key_maker import KeyMaker


class SyncedSignalSet(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self):
        self.synced_signals = dict()
        self.key_maker = KeyMaker('SSN')


    def append_synced_signal(self, synced_signal_obj, create_new_key=False):
        if create_new_key:
            idx = self.key_maker.get_new()
            while idx in self.synced_signals.keys():
                idx = self.key_maker.get_new()

            synced_signal_obj.idx = idx

        self.synced_signals[synced_signal_obj.idx] = synced_signal_obj
        self.synced_signals = dict(sorted(self.synced_signals.items()))


    def remove_synced_signal(self, synced_signal_obj):
        self.synced_signals.pop(synced_signal_obj.idx)


    def get_signal_list(self):
        signal_list = [] 
        for synced_signal in self.synced_signals.values():
            signal_list = signal_list + synced_signal.signal_set.to_list()

        return signal_list
    
    def remove_data(self, ss):
        self.synced_signals.pop(ss)
    