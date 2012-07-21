all: dynamics.so

dynamics.so: dynamics_setup.py dynamics.pyx
	python2.7 dynamics_setup.py build_ext --inplace --force
