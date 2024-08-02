# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import os

from isaacsim import AppFramework

argv = [
    "--empty",
    "--ext-folder",
    f'{os.path.abspath(os.environ["ISAAC_PATH"])}/exts'
    "--ext-folder",
    f'{os.path.abspath(os.environ["ISAAC_PATH"])}/extscache',
    "--ext-folder",
    f'{os.path.abspath(os.environ["ISAAC_PATH"])}/extsPhysics',
    "--no-window",
    "--/app/asyncRendering=False",
    "--/app/fastShutdown=True",
    "--enable",
    "omni.usd",
    "--enable",
    "omni.kit.uiapp",
    "--enable",
    "omni.anim.people"
]

# startup
app = AppFramework("test_app", argv)

import omni.usd

stage_task = asyncio.ensure_future(omni.usd.get_context().new_stage_async())

while not stage_task.done():
    app.update()

print("exiting")
app.close()
