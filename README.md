# Autosiding
Model rail CDU autosiding controller with optical trigger

Usage: 
Unit boots in auto mode with siding 1 active.  sidings advance automatically when sensors are triggered in sequence. 
A short button push in auto-mode will also advance the siding.
A long button push (3 sec) will lock a particular siding open.  Another long push will take you back to auto mode on that siding.
A short push in lock-mode will advance to the next siding, keeping that locked open.

Version 4 ported to PIC16F88, the 84 version used an external crystal.  The 88 verison frees up the ports for use.
