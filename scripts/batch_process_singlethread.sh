#!/bin/bash
for c in $(seq 0 0)
do
	for m in $(seq 0 0)
	do
		for u in $(seq 0 7)
		do
   			echo "c $c  m $m  u $u"
			#../build/ProcessMesh_compare 2 $c $m $u
			python prepare_data.py $1 $2 $c $m $u
		done
	done
done