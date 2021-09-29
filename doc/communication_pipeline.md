#:frog:Ridgeback + iiwa communication pipeline

###:purple_heart:Used topics:

Format: 
 
    /iiwa_ridgeback_communicaiton/sender

1. /iiwa_ridgeback_communicaiton/iiwa
2. /iiwa_ridgeback_communicaiton/ridgeback

###:purple_heart:Used message

Type:

    std_msgs <String>

Content:
```commandline
number: instruction number
status: 
   start: start instruction
   executing: doing instructions
   done: done executing instruction
   error: error occured during execution
   waiting: done instruction and waiting for the other robot to send done signal
```

For example: 

    /iiwa_ridgeback_communicaiton/ridgeback
    4 done
meaning: 

ridgeback is done doing instruction 4

###:purple_heart:Pipeline
| Ridgeback | iiwa |
| ------------- | ------------- | 
| 0 start | 0 waiting |
| 0 executing | 0 waiting |
| 0 done | 0 waiting |
| 0 waiting | 0 start |
| 0 waiting | 0 executing |
| 0 waiting | 0 done |
| 1 start | 1 waiting |
