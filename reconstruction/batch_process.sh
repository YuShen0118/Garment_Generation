#!/bin/bash
for c in $(seq 2 13)
do
	for m in $(seq 0 9)
	do
		for u in $(seq 0 249)
		do
   			echo "c $c  m $m  u $u"
   			build/ProcessMesh $c $m $u
		done
	done
done