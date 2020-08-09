#!/bin/bash
for c in $(seq 1 104)
do
	for m in $(seq 0 10)
	do
		for u in $(seq 0 31)
		do
   			echo "c $c  m $m  u $u"
			u8=`expr 8 \* $u`
			id0=`expr 0 + $u8`
			id1=`expr 1 + $u8`
			id2=`expr 2 + $u8`
			id3=`expr 3 + $u8`
			id4=`expr 4 + $u8`
			id5=`expr 5 + $u8`
			id6=`expr 6 + $u8`
			id7=`expr 7 + $u8`
			#../build/ProcessMesh_test_big 0 $c $m $id0 &
			#../build/ProcessMesh_test_big 0 $c $m $id1 &
			#../build/ProcessMesh_test_big 0 $c $m $id2 &
			#../build/ProcessMesh_test_big 0 $c $m $id3 &
			#../build/ProcessMesh_test_big 0 $c $m $id4 &
			#../build/ProcessMesh_test_big 0 $c $m $id5 &
			#../build/ProcessMesh_test_big 0 $c $m $id6 &
			#../build/ProcessMesh_test_big 0 $c $m $id7

			python prepare_data.py $1 $2 $c $m $id0 &
			python prepare_data.py $1 $2 $c $m $id1 &
			python prepare_data.py $1 $2 $c $m $id2 &
			python prepare_data.py $1 $2 $c $m $id3 &
			python prepare_data.py $1 $2 $c $m $id4 &
			python prepare_data.py $1 $2 $c $m $id5 &
			python prepare_data.py $1 $2 $c $m $id6 &
			python prepare_data.py $1 $2 $c $m $id7
			wait
		done
	done
done