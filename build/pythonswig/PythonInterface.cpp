#include <Python.h>
#include <PythonInterface.h>

#include <stdio.h>

using namespace std;

VisionCore* PythonInterface::CORE_INSTANCE = NULL;

void PythonInterface::Init() {
  Py_SetProgramName("pythonswig");
  const char *chome = getenv("NAO_HOME");
  string home;
  if(chome) home = chome;
  string scriptPath = home + "/core/python:/home/nao/python";
  string modulePath = home + "/build/pythonswig:/home/nao/bin";
  string rootPath = home + "/naoqi/lib/python2.7:/usr/lib/python2.7";
  string libPath = home + "/naoqi/crosstoolchain/atom/sysroot/lib/python2.7:/lib/python2.7";


  string path = "PYTHONPATH=";
  path += scriptPath + ":";
  path += modulePath + ":";
  path += libPath + ":";
  path += rootPath;
  putenv((char*)path.c_str());

  Py_Initialize();
  PyRun_SimpleString(
    "import pythonswig_module\n"
    "pythonswig_module.PythonInterface().CORE_INSTANCE.python_.python_ok_ = False\n"
    "from init import *\n"
    "init()\n"
    "pythonswig_module.PythonInterface().CORE_INSTANCE.python_.python_ok_ = True\n"
  );
}

void PythonInterface::Import(string module) {
  string command = "import ";
  command += module;
  PyRun_SimpleString((char*)command.c_str());
}

void PythonInterface::Execute(string command) {
  PyRun_SimpleString((char*)command.c_str());
}

void PythonInterface::Finalize() {
  Py_Finalize();
}
