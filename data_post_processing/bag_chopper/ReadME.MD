bag chopper
- takes 3 arguments
1. bags.txt -- contains list of bag files
2. timestamps.json --contains ending timestamps of every split. Check example file for format
3. -- topics.json -- contains topics to split

How it works ?
Lets say we have `x` bags in x lines in bags.txt file.
We have a suffix for each bag in timestamps.json eg. `bag0, bag1, ....`
for each suffix in timestamps.json, we have a dictionary which has partition labels as keys and timestamps (in ns) as values.
topics.json will have keys for topic partitions and values being list of topics to include in that partition. eg. a partition named camera can have multiple topics in camera partition. eg. `"/cam_sync/cam0/camera_info","/cam_sync/cam0/image_raw/compressed","/cam_sync/cam0/meta","/cam_sync/cam1/camera_info","/cam_sync/cam1/image_raw/compressed","/cam_sync/cam1/meta"`. This is common for all files

So, if we have 5 bag files, and 10 partitions in timestamps.json for each bag file with the timestamps, and topics list, we will get a bagfiles named

`bag<i>_<timestamp_partition_name>_<topic_partition_name>`

eg. `bag0_highbay_lidarcam`

If you will be running for the bag1, use following command
```
python3 bag_chopper.py bag1.txt --topics topics_all.json --timestamps timestamps1.json 
```