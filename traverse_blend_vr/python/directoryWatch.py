# -*- coding: utf-8 -*-
"""
Created on Fri Dec 18 10:29:40 2015

@author: eruff

"""
import sys

sys.path.append("/usr/local/lib/python3.4/dist-packages/pyinotify-0.9.6-py3.4.egg") 
import pyinotify

from time import sleep
import asyncore


class EventHandler(pyinotify.ProcessEvent):
    
    def process_IN_CREATE(self, event):
        # load new lib
        print ("Creating:", event.pathname)

    def process_IN_DELETE(self, event):
        # remove lib
        print ("Removing:", event.pathname)

    def process_IN_MODIFY(self, event):
        # reload map
        print ("Modified:", event.pathname)

    def process_IN_CLOSE_WRITE(self, event):
        # reload map
        print ("Close written:", event.pathname)

    def process_IN_MOVED_TO(self, event):
        # load pathname
        # delet src_pathname
        
        print ("Moved to: ", event.pathname,"from",event.src_pathname,event.name[:-6])
    def process_IN_MOVED_FROM(self, event):
        return
        
def main(pathToWatch):
    wm = pyinotify.WatchManager()  # Watch Manager
    
    mask = pyinotify.IN_DELETE | pyinotify.IN_CREATE | pyinotify.IN_MODIFY | pyinotify.IN_CLOSE_WRITE | pyinotify.IN_MOVED_TO | pyinotify.IN_MOVED_FROM # watched events
    
    
    notifier = pyinotify.AsyncNotifier(wm, EventHandler())
    wm.add_watch('/tmp/directorywatch', mask)
    
    
    
    while(True):
        asyncore.poll()
        sleep(2)
    notifier.stop()
    
    

if __name__ == "__main__":
    main("/tmp/directorywatch")