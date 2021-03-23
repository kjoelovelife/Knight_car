#!/usr/bin/env python
import logging
logging.basicConfig()
logger = logging.getLogger(__name__)


class MyExc(Exception):
	pass

def main():
	try:
		import sys
		args = sys.argv[1:]
		if len(args) != 1:
			msg = 'Expected one argument, got %r.' % args
			raise MyExc(msg)

		go(args[0])

	except MyExc as e:
		logger.error(e)
		sys.exit(-2)
	except Exception as e:
		import traceback
		logger.error(traceback.format_exc(e))
		sys.exit(-1)


def go(scuderia):
	logger.info('This script generates "machines" from "scuderia".')

	scuderia_contents = read_scuderia(scuderia)
	machines_contents = create_machines(scuderia_contents)
	print(machines_contents)
	logger.info('Successful.')
	
def create_machines(scuderia_contents):
	""" Returns XML string """
	
	start = """<!-- DO NOT MODIFY: this is autogenerated --> 
<launch>
   <arg name="env_script_path" default="~/Knight_car/ros_version/melodic/environment.sh"/>
	"""
	
	def make_line(name):
		space = " "*(13-len(name))
		p = """<machine name="%s"  %s address="%s.local" %s user="ubuntu" env-loader="$(arg env_script_path)"/>"""
		return p % (name, space, name, space)

	names = sorted(scuderia_contents)

	s = start
	for name in names:

		s += '\n   ' + make_line(name)
	s += '\n</launch>\n'
	return s

def read_scuderia(scuderia):
	""" Returns the contents of scuderia as a dict, with validated fields."""
	import os
	
	if not os.path.exists(scuderia):
		msg = 'Could not find file %r.' % scuderia
		raise Exception(msg) 
	
	import yaml
	
	yaml_string = open(scuderia).read()
	
	try:
		values = yaml.load(yaml_string)
	except yaml.YAMLError as e:
		msg = 'Yaml file is invalid:\n---\n%s' % e
		raise MyExc(msg)

	if not isinstance(values, dict):
		msg = 'Invalid content: %s' % values
		raise MyExc(msg)

	n = len(values)

	names = ", ".join(sorted(values))
	logger.info('I found %d Duckiebots in scuderia: %s.' % (n, names))

	for k, value in values.items():
		check_good_name(k)
		
	return values


def check_good_name(name):
	if not name.lower() == name:
		msg = 'Name must be lowercase. %r is not a good name.' % name
		raise MyExc(msg)
	
	


# ## colored loggin

logger.setLevel(logging.DEBUG)
def add_coloring_to_emit_ansi(fn):
    # add methods we need to the class
    def new(*args):
        levelno = args[1].levelno
        if(levelno >= 50):
            color = '\x1b[31m'  # red
        elif(levelno >= 40):
            color = '\x1b[31m'  # red
        elif(levelno >= 30):
            color = '\x1b[33m'  # yellow
        elif(levelno >= 20):
            color = '\x1b[32m'  # green
        elif(levelno >= 10):
            color = '\x1b[35m'  # pink
        else:
            color = '\x1b[0m'  # normal

        args[1].msg = color + str(args[1].msg) + '\x1b[0m'  # normal
        return fn(*args)
    return new

import platform
if platform.system() != 'Windows':
    emit2 = add_coloring_to_emit_ansi(logging.StreamHandler.emit)
    logging.StreamHandler.emit = emit2


if __name__ == '__main__':
	main()
