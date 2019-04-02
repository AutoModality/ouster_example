#!/bin/bash
nc 192.168.1.55 7501 < ouster-512x20
sleep 1
nc 192.168.1.55 7501 < ouster-reinit
