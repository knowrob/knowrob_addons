# rs_openease

## Perception Planning
RoboSherlock uses Apache UIMA (Unstructured Information Management Architecture) to process the incoming sensor data with image processing algorithms, called experts. 
The selection and ordering of these experts (e.g. a 'pipeline') is usally hard coded by the developer of a robotic system. 
With RoboSherlock, you can automatically generate these pipelines based on the query you send to RoboSherlock and an Ontology about all implemented experts.
For example, you can ask the system to create a pipeline to retrieve the color and shape attribute of objects in front of your camera:
```
?- build_pipeline_from_predicates([shape,color],S).
S = [rs_components:'CollectionReader',rs_components:'NormalEstimator',
rs_components:'ImagePreprocessor',rs_components:'PointCloudClusterExtractor',
rs_components:'FlatObjectAnnotator',rs_components:'SacModelAnnotator',
rs_components:'PrimitiveShapeAnnotator',rs_components:'ClusterColorHistogramCalculator'].
```
### Usage
* Install RoboSherlock. Currently, you MUST use the pipeline_modification branch: https://github.com/code-iai/iai_robosherlock/tree/pipeline_modification
* Start RoboSherlock with: 
```rosrun iai_rs_cpp rs_runAE -pipelinegen -visualizer test.xml```. Note: test.xml is the name of one of your existing analysis engines.
* Start the json_prolog service for the pipeline planning: 
```roslaunch knowrob_robosherlock knowrob_robosherlock.launch```
* Start the query_runAE_service.py shell to send queries to RoboSherlock with:
```python `rospack find iai_rs_cpp`/scripts/query_runAE_service.py```
* In the loaded shell, ask RoboSherlock to execute a pipeline to detect the Shape and Color Property of objects:
```rs_query$ detect SHAPE,COLOR```
Please note, that you can use auto completion in the query_runAE_service.py shell.

### Useful commands
 * RoboSherlock is capable of executing multiple generated pipelines in sequence, if the requested properties can be detected by different combinations of annotators in a pipeline. Use the 'solution' command in the query_runAE_service.py shell to define wether you want to try out every possible combination (command: solution all) or the one with the most annotators that compute the requested properties (command: solution single).
 * By default, you will load a couple of test objects with the knowrob_robosherlock.launch file. Knowledge about these objects is asserted in a Knowledge Base, which enables RoboSherlock to plan Pipelines specifically for them. Use the 'detectobj' command to build a pipeline for a given object. The defined objects can be seen by using the tab completion feature when you've entered 'detectobj' plus a space in the query_runAE_service.py shell. The selected solution mode, as discussed above, will also affect the 'detectobj' command.

### Unit tests
We've implemented unit tests for the pipeline generation which can be found in prolog/knowrob_robosherlock.plt. These tests can be executed with:
    ```rosrun rosprolog rosprolog-test knowrob_robosherlock```
The test may also be a good entry point for you to see how you can use the pipeline planning for your own projects.
Note: You might not be able to execute the unit tests if you are using the current version of KnowRob (January 2015) and Ubuntu 14.04 with SWI-prolog 6.6.
If you get an error when you try to execute the tests, look into knowrob_common/prolog/plunit.pl and look for the declare_module statement in line 248.
Change it from:

    '$declare_module'(Name, Context, File, Line, false),
to

    '$declare_module'(Name, test, Context, File, Line, false),
(see http://sourceforge.net/p/yap/plunit/ci/d215c5bf2b83b4f2766f204d7464a56ea8e914e2/ for details).
