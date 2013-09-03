#!/usr/bin/env python
import sys, os, subprocess, re

def generateI(listfile, srcroot, ifile):
  f = open(listfile)
  lines = ["%s/%s" % (srcroot, line.rstrip()) if line.rstrip() else "" for line in f]
  o = open(ifile, 'w')
  o.write("%module pythonswig_module\n")
  o.write('%include "std_string.i"\n')
  o.write('%include "std_vector.i"\n')
  o.write("%{\n")
  for line in lines:
    if not line: continue
    o.write('  #include "%s"\n' % line)
  o.write("%}\n\n")
  for line in lines:
    if not line: continue
    o.write('%%include "%s"\n' % line)
  o.write('%template(vector2_float) Vector2<float>;\n')
  o.write('%template(vector3_float) Vector3<float>;\n')
  o.write('%template(vector_float) std::vector<float>;\n')
  o.close()

def runSwig(ifile, cppfile):
  subprocess.check_call('swig2.0 -w362,389,401,454,462,503 -python -c++ -o %s %s' % (cppfile, ifile),shell=True)

if __name__ == '__main__':
  print "Generating swig wrapper"
  args = sys.argv[1:]
  listfile = args[0]
  srcroot = args[1]
  ifile = args[2]
  cppfile = args[3]
  generateI(listfile, srcroot, ifile)
  runSwig(ifile, cppfile)
