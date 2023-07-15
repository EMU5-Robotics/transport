**Initial Communication**

`1: `Brain -> Co-processor
+ Port State (22 * 1-byte)
+ TriPort Present (8-bits)

`2: `Co-processor -> Brain
+ Ack (1-byte)

**Constant Communication**
Both packets `3` & `4` should be sent every 5ms from the source to the sink.

`3: `Brain -> Co-processor
+ Competition State (1-byte)
+ Controller Inputs (6-bytes)
+ Encoders (16 * 4-bytes)
+ Motor State (14 * 3-bytes)
+ Miscellaneous (4-bytes)

`4: `Co-processor -> Brain
+ Motor Power (14 * 1-byte)
+ TriPort Toggle (8-bits)
