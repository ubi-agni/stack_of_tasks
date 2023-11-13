## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=[
        "stack_of_tasks",
        "stack_of_tasks.robot_model",
        "stack_of_tasks.ui.utils",
        "stack_of_tasks.ui.tabs",
        "stack_of_tasks.ui.widgets",
        "stack_of_tasks.ui.traits_mapping.editor_entry",
        "stack_of_tasks.ui.traits_mapping",
        "stack_of_tasks.ui.traits_mapping.painter_entry",
        "stack_of_tasks.ui.model",
        "stack_of_tasks.ui",
        "stack_of_tasks.ui.property_tree",
        "stack_of_tasks.ui.generated",
        "stack_of_tasks.utils",
        "stack_of_tasks.tasks",
        "stack_of_tasks.ref_frame",
        "stack_of_tasks.parameter",
        "stack_of_tasks.solver",
        "stack_of_tasks.plot",
        "stack_of_tasks.marker",
    ],
    package_dir={"": "python"},
)
setup(**setup_args)
