all: prueba

testc: 
	echo out
rk4:
	gcc -c rk4_2D_3D.c
common:
	gcc -c common.c -lm
msfm2d_mod:
	gcc -c msfm2d_MOD.c -lm
msfm3d_mod:
	gcc -c msfm3d_MOD.c -lm
planner: 
	gcc -c planner.c -lm
FFM2: msfm2d_mod msfm3d_mod
	gcc -c FMM2.c -lm
prueba: msfm2d_mod msfm3d_mod FFM2 planner rk4 common
	gcc prueba.c ./FMM2.o ./planner.o ./msfm2d_MOD.o ./msfm3d_MOD.o ./rk4_2D_3D.o ./common.o -lm
clean: clear
	clear
clear:
	rm -rf ./*.o; 