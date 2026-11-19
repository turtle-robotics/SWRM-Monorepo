# PubSubClient v2.8 — one local accessor

Source: https://github.com/knolleary/pubsubclient/tree/v2.8

`src/PubSubClient.cpp` is unchanged upstream source. The only header change is
the public `messageRetained()` accessor, valid only at callback entry, before
publishing (which reuses the packet buffer). It exposes the existing MQTT
PUBLISH RETAIN bit so the unchanged `onMessage(char*, byte*, unsigned int)`
callback signature can reject retained square/spin requests. No wire protocol,
connection logic, topic handling or fragmentation behavior was changed.

The lead used `knolleary/PubSubClient@^2.8`; this local MIT-licensed copy fixes
the tested base at v2.8 and keeps the required metadata extension reviewable.
Do not install another PubSubClient alongside it or remove the accessor without
replacing retained-start protection. Firmware builds verify the actual library;
host packet tests exercise the real library's retained/nonretained callback path.
