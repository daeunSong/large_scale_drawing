# Ridgeback + iiwa communication pipeline

### Used topics:

Format: 
 
    /iiwa_ridgeback_communicaiton/sender

1. /iiwa_ridgeback_communicaiton/iiwa
2. /iiwa_ridgeback_communicaiton/ridgeback

### Used message

Type:

    std_msgs <Int32>

### Status

| Robot     | Status | Meaning                                       |
|-----------|--------|-----------------------------------------------|
| iiwa      | 0      | Not working - waiting for Ridgeback to finish |
|           | 1      | Working - drawing                             |
|           | 2      | Halting to change color                       |
| Ridgeback | 0      | Not working - waiting for iiwa to finish      |
|           | 1      | Working - moving to the next range            |
