# refactor
import re, os

def convert_pos_to_named(filename_i, filename_o, func_name):
    """
        Convert positional arguments to named arguments for further refactoring under following assumptions.
        1. Refactoring functions defined in GridLayoutGenerator.py
        2. GridLayoutGenerator is instantiated as laygen
    """
    # GridLayoutGenerator parameters
    laygen_path="../../GridLayoutGenerator.py"
    laygen_instance="laygen"

    # read function definition
    trig=0 #trigger for multiline definitions
    with open(laygen_path, 'r') as f:
        lines_s = f.readlines()
        # read function definition
        for l in lines_s:
            if 'def '+func_name+'(' in l:  # end of function call
                l_token_arg = [] #tokens for arguments
                trig = 1  #trig to readout arguments over multiple lines
                depth = 0 #depth variable to figure out argument definitions
                print("function " + func_name + " definition detected. code snapshot: " + l[:-1])
            if trig == 1:
                s_buf=''
                for c in l:
                    if c==' ' and s_buf=='': #ignore spaces between commas and indents
                        pass
                    else:
                        if c==')' or c==']': #exit bracket decrease depth
                            if depth == 1: #end of definition
                                if s_buf=='': #some functions have something like , ):
                                    pass
                                else:
                                    l_token_arg.append(s_buf)
                                    s_buf = ''
                                trig = 0
                            depth -= 1
                        if c==',' and depth == 1: #if the comma is argument splitter,
                            l_token_arg.append(s_buf)
                            s_buf=''
                        else:
                            if depth >= 1:
                                s_buf += c
                        if c=='(' or c=='[': #go inside bracket. increase depth
                            depth += 1
                if trig == 0:
                    l_token_arg=l_token_arg[1:] #exclude self
                    for i, item in enumerate(l_token_arg):
                        if '=' in item: #has a default value
                            l_token_arg[i] = re.split("=", item)[0]
                    if trig == 0:
                        print("function "+func_name+" definition loaded. arguments: "+str(l_token_arg))

    # read source code
    with open(filename_i, 'r') as f:
        lines_i = f.readlines()
    print("file " + filename_i + " loaded")

    # refactor
    trig = 0  # trigger for multiline call
    lines_o = [] # output buffer
    for i, l in enumerate(lines_i):
        if laygen_instance + '.' + func_name in l:
            trig = 1
            print("function " + func_name + " call detected in file: "+ filename_i + ", in line:"+ str(i) +" code snapshot: " + l[:-1])
        if trig == 1:
            l_refac = '' #refactored line
            s_buf = ''   #string buffer to store arguments
            arg_index = 0 #argument index to map positional arguments
            for c in l:
                trig_copy = 1  # trigger to copy c to l_refac without modifications
                trig_refac_arg = 0 # trigger to refactor arguements after readout
                if c == ' ' and s_buf == '':  # ignore spaces between commas and indents
                    pass
                else:
                    if c == ')' or c == ']':  # exit bracket decrease depth
                        if depth == 1:  # end of function call
                            if s_buf == '':  # some functions have something like , ):
                                pass
                            else:
                                trig_refac_arg = 1
                            trig = 0 #end of function call
                        depth -= 1
                    if c == ',' and depth == 1:  # if the comma is argument splitter,
                        trig_refac_arg = 1
                    else:
                        if depth >= 1:
                            s_buf += c
                            trig_copy = 0
                    if c == '(' or c == '[':  # go inside bracket. increase depth
                        depth += 1
                if trig_refac_arg == 1: #s_buf filled, do refactoring
                    if '=' in s_buf: #named argument, just copy and paste
                        l_refac += s_buf
                    else: #positional argument!
                        l_refac += l_token_arg[arg_index] + ' = ' + s_buf
                        arg_index += 1 #move to next argument
                    s_buf = '' #flush s_buf
                if trig_copy == 1:
                    l_refac += c
            print("   before refactoring: "+l)#+l[:-1]) #remove newline for neat plotting
            print("    after refactoring: "+l_refac)#+l_refac[:-1])
            lines_o.append(l_refac) #
        else: #normal codes, just copy and paste
            lines_o.append(l)

    # write source code
    with open(filename_o, 'w') as f:
        for l in lines_o:
            f.write(l)

if __name__ == '__main__':
    '''
    #single run example
    filename_i = "../logic/logic_templates_layout_generator.py"
    filename_o = "../logic/logic_templates_layout_generator_refactored.py"
    # func_name = "pin_from_rect"
    func_name = "relplace"
    convert_pos_to_named(filename_i=filename_i, filename_o=filename_o, func_name=func_name)
    '''

    #massive run over multiple directories, functions
    dir_list=[".", "../adc_sar/", "../golden/", "../logic/", "../serdes/", "../../labs"]
    func_list=["pin_from_rect"]
    for dir in dir_list:
        file_list=os.listdir(dir)
        for file in file_list:
            if file.endswith('_layout_generator.py'):
                filename=dir+file
                for func in func_list:
                    convert_pos_to_named(filename_i=filename, filename_o=filename, func_name=func)
