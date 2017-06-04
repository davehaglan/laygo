# SAR ADC generator

This section describes how to generate a SAR ADC schematic / layout
using BAG and laygo.

## Installation

1. Set up BAG environment for the technology in use. This is usually
done by cloning technology repo provided by BAG developers. For example,
cds_ff_mpt environment can be set up by running following commands.

    ```
    $ git clone git@github.com:ucb-art/BAG2_cds_ff_mpt.git
    $ cd BAG2_cds_ff_mpt
    $ git submodule init
    $ git submodule update
    $ git submodule foreach git pull origin master
    ```

    > Note: for BWRC users, set up repos are provided under proper NDA.

run laygo/labs/lab2_a_gridlayoutgenerator_constructtemplate.py

run laygo/generators/logic/logic_templates_layout_generator.py


2. Set setting up files, run virtuoso and BAG.

3. Create an empty library called (tech)_logic_templates. For
example, for **cds_ff_mpt** technology, the library name will be
**cds_ff_mpt_logic_template**.

4. Run this command to generate logic layouts.

    ```python
    run laygo/generators/logic/logic_templates_layout_generator.py
    ```

5. Open (tech)_logic_templates library and check if cells are
generated.

    ![logic_lib](images/logic_lib.png)

6. Run this command to generate a serializer layout.

    ```python
    run laygo/generators/serdes/serdes_layout_generator.py
    ```
