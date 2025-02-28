'''
linting util to reformat f-string logging to lazy logging
'''
from __future__ import print_function

import os
import fileinput
import re
import mmap
import sys

# for line in (line.rstrip() for line in fileinput.FileInput("file_path", inplace=1)):
#     print(line)
#     print(line, file=sys.stderr)

TEST_STRING = '  logger.info(f"timestamps: overall_step_time_ms - {round(overall_step_time_ms, 2)}ms | step_latency_ms - {round(step_latency_ms, 2)}ms")'

logging_re = re.compile(r'([# ]*)logger\.([a-z]+)\(f(.+)\)')

# for (root,_,files) in os.walk('opencda', topdown=True):
#     for file in files:
#         if file.endswith('.py'):
file = 'vehiclesim.py'
root = os.getcwd()
if True:
            print(f"scanning for f-string logging in {file}")
            with open(os.path.join(root, file), 'r+b') as f:
                # Create a memory map of the file
                try:
                    mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
                except ValueError:
                    print(f'ValueError in {file}')
                    #continue
                else:
                    if mm.find(b'logger.') != -1:
                        print(f'need to fix logging found in {file}')
                        mm.close()
                    else:
                        print(f'no logging found in {file}')
                        mm.close()
                        #continue

            print(f'fixing logging in {file}')
            for line in (line.rstrip() for line in fileinput.FileInput(os.path.join(root, file), inplace=1)):
                rex = re.search(logging_re, line)
                if not rex:
                    print(line)

                else:
                    leading_spaces = rex.group(1)
                    log_level = rex.group(2)
                    f_string = rex.group(3)

                    arg_list = []
                    for grp in re.finditer(r'{([^}]+)}', f_string):
                        arg_list.append(grp.group(1))

                    lazy_string = re.sub(r'{[^}]+}', '%s', f_string)

                    arg_string = ''
                    if len(arg_list) > 0:
                        arg_string = ', ' + ', '.join(arg_list)

                    new_line = f'{leading_spaces}logger.{log_level}({lazy_string}{arg_string})'
                    print(new_line)