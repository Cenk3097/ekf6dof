run_test: run_test.cc
	g++ run_test.cc -o run_test `pkg-config --libs --cflags opencv`


