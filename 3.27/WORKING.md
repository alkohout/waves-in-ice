24/8/2022
trying to fix run_test
I suspect it is to do with pointers problem
I've discovered that decimate in and out are pointers, but in and out aren't pointers when decimate used in test code. 
Trying to work out how to change that

working in peak_freq. Trying to find sig fault cause. I'm guessing its a memory problem.
fixed. I needed to add malloc in cycle_decimate

I've changed i back to 13 in test_sample_length and am getting seg faults again. I suspect it is in start integration test and 
needs malloc added somewhere.

I've moved it back to 12. Couldn't solve it. Have added malloc to disp and data in test_integration

Now have run test working! YAY. 
Now need to fix Hs test. I cant get 3.26 to return 0.119 (as it should be)

