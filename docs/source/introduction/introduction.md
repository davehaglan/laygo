# LAYGO - LAYout with Gridded Objects 

Laygo is an add-on to [BAG2](https://github.com/pkerichang/BAG_framework)
framework for layout generation, licensed under the BSD license.
With laygo, layout elements and parameters are abstracted as templates
and grids, and the layout job can be done without dealing with complex
design rules in advanced CMOS technology.
Layout process is abstracted and scripted in python, and with the help
of BAG API, laygo provides higher productivity and process portability
than conventional custom IC design flows.

![laygo](images/laygo_concept.png)

## Installation and Quick Start
1. Install BAG2 (not necessary if the GDS flow is used)
2. Clone laygo repo
    ```
    $ git clone git@github.com:ucb-art/laygo.git
    ```
3. Prepare following etup files for your technology.
    * **laygo_config.yaml** - contains general technology information.

        An example file can be found here: [labs/laygo_config.yaml](https://github.com/ucb-art/laygo/blob/master/labs/laygo_config.yaml)
    * **(technology_name).layermap**(optional) - layer mapping file only
    for the GDS flow. Usually layermap files can be found in your PDK
    library.

        An example file can be found here: [labs/laygo_faketech.layermap](https://github.com/ucb-art/laygo/blob/master/labs/laygo_faketech.layermap)
    * **primitive template and grid database** : laygo keeps template
    and grid information in yaml files. Users can construct yaml files
    by themselves, or files can be provided under NDA.

        Examples files can be found here (for laygo_faketech, used in the GDS flow):
        [labs/laygo_faketech_microtemplates_dense_templates.yaml](https://github.com/ucb-art/laygo/blob/master/labs/laygo_faketech_microtemplates_dense_templates.yaml),
        [labs/laygo_faketech_microtemplates_dense_grids.yaml](https://github.com/ucb-art/laygo/blob/master/labs/laygo_faketech_microtemplates_dense_grids.yaml)

        An example script that consturcts the yaml database is here:
        [labs/lab2_a_gridlayoutgenerator_constructtemplate.py](https://github.com/ucb-art/laygo/blob/master/labs/lab2_a_gridlayoutgenerator_constructtemplate.py)

    Example setup files for generic technologies are released for
    reference, which can be found here:

        * [cds_ff_mpt](git@github.com:ucb-art/BAG2_cds_ff_mpt.git)
        * [NCSU FreePDK45](git@github.com:ucb-art/BAG2_freePDK45.git)

    For BWRC users, default setup files for real technologies can be
    provided under proper NDA, as a form of git repository.

4. For a toy example, launch ipython and run the GDS tutorial script
    [quick_start_GDS.py](https://github.com/ucb-art/laygo/blob/master/quick_start_GDS.py).
    ```
    $ start_bag.sh    (or ipython)
    > cd laygo
    > run quick_start_GDS.py
    ```
    It will create a nand gate layout and save it to output.gds.

    ![qs_nand](images/laygo_quickstart.png)

    [KLayout](http://www.klayout.de/) was used for gds display. Detailed
    explanations on the tutorial script can be found
    [here](https://ucb-art.github.io/laygo/tutorial/tutorial_GDS.html).
    You can also export the layout to the BAG framework. Refer to
    [this document](https://ucb-art.github.io/laygo/tutorial/tutorial_BAG.html)
    for details.

5. For more practice, go over lab materials in [labs/](https://github.com/ucb-art/laygo/tree/master/labs).
Detailed instructions can be found in [lab readme](docs/labs.md).

6. More generator examples are being uploaded in [https://ucb-art.github.io/laygo/](https://ucb-art.github.io/laygo/) for reference.

## Documentations
Documents can be found at [https://ucb-art.github.io/laygo/](https://ucb-art.github.io/laygo/).

## Example Labs
Various lab modules are provided to guide the layout generation
procedure. Users are strongly recommended to finish all lab modules
before working on their designs. Labs modules can be found [here](docs/labs.md)

## Example Generators
Example generaters can be found [here](docs/generators.md).

## License
This project is licensed under the BSD License - check the
[LICENSE](LICENSE) file for details.

