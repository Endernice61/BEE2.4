"""Continuously moving belts, like in BTS.
"""
from __future__ import annotations

from collections import defaultdict

import attrs
from srctools import Keyvalues, Vec, Entity, Output, VMF, Matrix
from srctools.math import FrozenVec

import srctools.logger
from precomp import instanceLocs, template_brush, conditions, connections
from precomp.connections import ITEMS, Item
import consts

COND_MOD_NAME: str | None = None
LOGGER = srctools.logger.get_logger(__name__, alias='cond.conveyorBelt')

@attrs.define
class Marker:
    """A single node point."""
    ent: Entity = attrs.field(on_setattr=attrs.setters.frozen)
    name: str
    file: str
    pos: Vec
    item: Item
    orient: Matrix = attrs.field(init=False, on_setattr=attrs.setters.frozen)

    # noinspection PyUnresolvedReferences
    @orient.default # type: ignore
    def _init_orient(self) -> Matrix:
        """We need to rotate the orient, because items have forward as negative X."""
        rot = Matrix.from_angstr(self.ent['angles'])
        return Matrix.from_yaw(180) @ rot


@conditions.make_result('ConveyorBelt')
def res_conveyor_belt(vmf: VMF, inst: Entity, res: Keyvalues) -> None:
    """Create a conveyor belt.
    * Options:
        * `New`: Are we generating a new conveyor belt type?
        * `EndInst`: Instance for the end of the conveyor belts.
          Added in precomp so instance names are the same.
        * `SegmentInst`: The instance for the conveyor belt segments.
        * `TrackInst`: The track the segments ride on.
        * `Speed`: The fixup or number for the belt speed.
        * `MotionTrig`: If set, a trigger_multiple will be spawned that
          `EnableMotion`s weighted cubes. The value is the name of the relevant filter.
        * `BeamKeys`: If set, a list of keyvalues to use to generate an env_beam
          travelling from start to end. The origin is treated specially - X is
          the distance from walls, y is the distance to the side, and z is the
          height.

    * Old Options:
        * `SegmentInst`: Generated at each square. (`track` is the name of the
          path to attach to.)
        * `TrackTeleport`: Set the track points so they teleport trains to the start.
        * `Speed`: The fixup or number for the train speed.
        * `MotionTrig`: If set, a trigger_multiple will be spawned that
          `EnableMotion`s weighted cubes. The value is the name of the relevant filter.
        * `EndOutput`: Adds an output to the last track. The value is the same as
          outputs in VMFs.
        `RotateSegments`: If true (default), force segments to face in the
          direction of movement.
        * `BeamKeys`: If set, a list of keyvalues to use to generate an env_beam
          travelling from start to end. The origin is treated specially - X is
          the distance from walls, y is the distance to the side, and z is the
          height.
        `RailTemplate`: A template for the track sections. This is made into a
          non-solid func_brush, combining all sections.
        * `NoPortalFloor`: If set, add a `func_noportal_volume` on the floor
          under the track.
        * `PaintFizzler`: If set, add a paint fizzler underneath the belt.

    """
    new_conveyor: bool = res.bool('New', False)

    if new_conveyor:
        # Generate our new conveyor instead

        mark1 = Marker(
            inst,
            inst["targetname"],
            inst["file"],
            inst.get_origin(),
            ITEMS[inst["targetname"]],
            )

        if not mark1.item.outputs:
            # Item has no outputs and is probably an end\
            #LOGGER.info("Conveyor Belt " + inst_name + " has no outputs")
            return

        LOGGER.info("Generating Conveyor Belt " + mark1.name)

        mark2: Marker | None = None

        for output in mark1.item.outputs:
            conn_item = output.to_item
            conn_inst = conn_item.inst
            if conn_inst['file'].casefold() == mark1.file:
                if mark2 is not None:
                    raise ValueError(f'Conveyor belt {mark1.name} has two connections!')
                mark2 = Marker(
                    conn_inst,
                    conn_inst["targetname"],
                    conn_inst["file"],
                    conn_inst.get_origin(),
                    conn_item,
                    )
                for conn_output in mark2.item.outputs:
                    if conn_output.to_item.name == mark1.item.name:
                        raise ValueError('Cyclical Conveyor Belt connection (ends are connected to eachother)!')
            else:
                raise ValueError(f'Conveyor Belt {mark1.name} connected to non-conveyor belt!')

        if mark2 is None:
            LOGGER.info(f"Conveyor belt {mark1} has no end instance")
            return

        mark1.item.delete_antlines()

        size_vec = abs((mark1.pos + (Vec(64, 0, 0) @ mark1.orient)) - (mark2.pos + (Vec(64, 0, 0) @ mark2.orient)))
        size: int = int((size_vec.x + size_vec.y + size_vec.z) / 128)
        #LOGGER.info("Belt Size: " + str(size))

        inst.fixup['$size'] = size

        # These checks are incredibly messy, simplify them later
        if mark1.orient.forward().axis() == 'x':
            if not mark1.pos.y == mark2.pos.y and not mark1.pos.z == mark2.pos.z:
                raise ValueError(f'Conveyor Belts are not in line (x axis) {mark1.pos} {mark2.pos}')
        if mark1.orient.forward().axis() == 'y':
            if not mark1.pos.x == mark2.pos.x and not mark1.pos.z == mark2.pos.z:
                raise ValueError(f'Conveyor Belts are not in line (y axis) {mark1.pos} {mark2.pos}')
        if mark1.orient.forward().axis() == 'z':
            if not mark1.pos.x == mark2.pos.x and not mark1.pos.y == mark2.pos.y:
                raise ValueError(f'Conveyor Belts are not in line (z axis) {mark1.pos} {mark2.pos}')
        if not mark1.orient.forward() == -mark2.orient.forward():
            raise ValueError(f'Conveyor Belts are not facing eachother {mark1.orient.forward()} {mark2.orient.forward()}')

        end_inst_file = instanceLocs.resolve_one(res['EndInst', ''], error=False)
        segment_inst_file = instanceLocs.resolve_one(res['SegmentInst', ''], error=False)
        track_inst_file = instanceLocs.resolve_one(res['TrackInst', ''], error=False)
    
        conditions.add_inst(
                vmf,
                targetname=mark1.name,
                file=end_inst_file,
                origin=mark1.pos,
                angles=mark1.orient,
            )
        
        conditions.add_inst(
                vmf,
                targetname=mark1.name,
                file=end_inst_file,
                origin=mark2.pos,
                angles=mark2.orient,
            )

        offset = 256
        track_name = conditions.local_name(inst, '&segment{}')
        track_start: Vec = mark1.pos + (Vec(offset, 0, 32) @  mark1.orient)
        track_end: Vec = mark2.pos + (Vec(offset, 0, 32) @ mark2.orient)

        norm = mark1.orient.up()

        if res.bool('rotateSegments', True):
            orient = Matrix.from_basis(x=mark1.orient.forward(), z=norm)
            inst['angles'] = orient.to_angle()
        else:
            orient = mark1.orient

        for index, pos in enumerate(track_start.iter_line(track_end, stride=128), start=1):
            # Don't place at the last point - it doesn't teleport correctly,
            # and would be one too many.
            if segment_inst_file and pos != track_end:
                seg_inst = conditions.add_inst(
                    vmf,
                    targetname=track_name.format(index),
                    file=segment_inst_file,
                    origin=mark1.pos, #spawn these at the same spot so they have the same lighting
                    angles=orient,
                )
                #seg_inst.fixup.update(inst.fixup)

        for index, pos in enumerate(mark1.pos.iter_line(mark2.pos, stride=128), start=1):
            conditions.add_inst(
                vmf,
                targetname=mark1.name + f'-track{index}',
                file=track_inst_file,
                origin=pos,
                angles=mark1.orient,
            )

        # Add the EnableMotion trigger_multiple seen in platform items.
        # This wakes up cubes when it starts moving.
        motion_filter = res['motionTrig', None]

        # Disable on walls, or if the conveyor can't be turned on.
        if norm != (0, 0, 1) or inst.fixup['$connectioncount'] == '0':
            motion_filter = None
        
        if motion_filter is not None:
            motion_trig = vmf.create_ent(
                classname='trigger_multiple',
                targetname=conditions.local_name(inst, 'enable_motion_trig'),
                origin=mark1.pos,
                filtername=motion_filter,
                startDisabled=1,
                wait=0.1,
            )
            motion_trig.add_out(Output('OnStartTouch', '!activator', 'ExitDisabledState'))
            # Match the size of the original...
            motion_trig.solids.append(vmf.make_prism(
                mark1.pos + Vec(72, -56, 58) @ orient,
                mark2.pos + Vec(-72, 56, 144) @ orient,
                mat=consts.Tools.TRIGGER,
            ).solid)

        push_speed = res['speed', None]
        if push_speed is None:
            push_speed = inst.fixup['$speed']
        if push_speed is not None and norm == (0, 0, 1):
            push_trig = vmf.create_ent(
                classname='trigger_push',
                targetname=conditions.local_name(inst, 'push'),
                spawnflags=4097,
                origin=mark1.pos,
                startDisabled=1,
                speed=int(push_speed)*128,
                wait=0.1,
            )
            push_trig.solids.append(vmf.make_prism(
                mark1.pos + Vec(64, -60, 59) @ orient,
                mark2.pos + Vec(-64, 60, 60) @ orient,
                mat=consts.Tools.TRIGGER,
            ).solid)

        # A brush covering under the platform.
        base_trig = vmf.make_prism(
            mark1.pos + Vec(64, 60, 50) @ orient,
            mark2.pos + Vec(-64, -60, 58) @ orient,
            mat=consts.Tools.INVISIBLE,
        ).solid

        vmf.add_brush(base_trig)

        # Make a paint_cleanser under the belt..
        if res.bool('PaintFizzler'):
            pfizz = vmf.create_ent(
                classname='trigger_paint_cleanser',
                origin=mark1.pos,
            )
            pfizz.solids.append(base_trig.copy())
            for face in pfizz.sides():
                face.mat = consts.Tools.TRIGGER

        mark2.ent.remove()
            
        # END OF NEW CONVEYOR BELTS
        #--------------------------
        return
    
    LOGGER.info("Generating old Conveyor Belt: " + inst['targetname'].casefold())

    move_dist = inst.fixup.int('$travel_distance')

    if move_dist <= 256:
        # There isn't room for a conveyor, so don't bother.
        inst.remove()
        return

    orig_orient = Matrix.from_angstr(inst['angles'])
    move_dir = Matrix.from_angstr(inst.fixup['$travel_direction']).forward()
    move_dir = move_dir @ orig_orient
    start_offset = inst.fixup.float('$starting_position')
    teleport_to_start = res.bool('TrackTeleport', True)
    segment_inst_file = instanceLocs.resolve_one(res['SegmentInst', ''], error=False)
    rail_template = res['RailTemplate', None]

    track_speed = res['speed', None]

    start_pos = Vec.from_str(inst['origin'])
    end_pos = start_pos + move_dist * move_dir

    if start_offset > 0:
        # If an oscillating platform, move to the closest side..
        offset = start_offset * move_dist * move_dir
        # The instance is placed this far along, so move back to the end.
        start_pos -= offset
        end_pos -= offset
        if start_offset > 0.5:
            # Swap the direction of movement..
            start_pos, end_pos = end_pos, start_pos
        inst['origin'] = start_pos

    norm = orig_orient.up()

    if res.bool('rotateSegments', True):
        orient = Matrix.from_basis(x=move_dir, z=norm)
        inst['angles'] = orient.to_angle()
    else:
        orient = orig_orient

    # Add the EnableMotion trigger_multiple seen in platform items.
    # This wakes up cubes when it starts moving.
    motion_filter = res['motionTrig', None]

    # Disable on walls, or if the conveyor can't be turned on.
    if norm != (0, 0, 1) or inst.fixup['$connectioncount'] == '0':
        motion_filter = None

    track_name = conditions.local_name(inst, 'segment_{}')
    rail_temp_solids = []
    last_track = None
    # Place tracks at the top, so they don't appear inside wall sections.
    track_start: Vec = start_pos + 48 * norm
    track_end: Vec = end_pos + 48 * norm
    for index, pos in enumerate(track_start.iter_line(track_end, stride=128), start=1):
        track = vmf.create_ent(
            classname='path_track',
            targetname=track_name.format(index) + '-track',
            origin=pos,
            spawnflags=0,
            orientationtype=0,  # Don't rotate
        )
        if track_speed is not None:
            track['speed'] = track_speed
        if last_track:
            last_track['target'] = track['targetname']

        if index == 1 and teleport_to_start:
            track['spawnflags'] = 16  # Teleport here..

        last_track = track

        # Don't place at the last point - it doesn't teleport correctly,
        # and would be one too many.
        if segment_inst_file and pos != track_end:
            seg_inst = conditions.add_inst(
                vmf,
                targetname=track_name.format(index),
                file=segment_inst_file,
                origin=pos,
                angles=orient,
            )
            seg_inst.fixup.update(inst.fixup)

        if rail_template:
            temp = template_brush.import_template(
                vmf,
                rail_template,
                pos,
                orient,
                force_type=template_brush.TEMP_TYPES.world,
                add_to_map=False,
            )
            rail_temp_solids.extend(temp.world)

    if rail_temp_solids:
        vmf.create_ent(
            classname='func_brush',
            origin=track_start,
            spawnflags=1,  # Ignore +USE
            solidity=1,  # Not solid
            vrad_brush_cast_shadows=1,
            drawinfastreflection=1,
        ).solids = rail_temp_solids

    if teleport_to_start and last_track is not None:
        # Link back to the first track...
        last_track['target'] = track_name.format(1) + '-track'

    # Generate an env_beam pointing from the start to the end of the track.
    try:
        beam_keys = res.find_key('BeamKeys')
    except LookupError:
        pass
    else:
        beam = vmf.create_ent(classname='env_beam')

        beam_off = beam_keys.vec('origin', 0, 63, 56)

        for prop in beam_keys:
            beam[prop.real_name] = prop.value

        # Localise the targetname so it can be triggered..
        beam['LightningStart'] = beam['targetname'] = conditions.local_name(
            inst,
            beam['targetname', 'beam']
        )
        del beam['LightningEnd']
        beam['origin'] = start_pos + Vec(
            -beam_off.x, beam_off.y, beam_off.z,
        ) @ orient
        beam['TargetPoint'] = end_pos + Vec(
            +beam_off.x, beam_off.y, beam_off.z,
        ) @ orient

    # Allow adding outputs to the last path_track.
    if last_track is not None:
        for prop in res.find_all('EndOutput'):
            output = Output.parse(prop)
            output.output = 'OnPass'
            output.inst_out = None
            output.comma_sep = False
            output.target = conditions.local_name(inst, output.target)
            last_track.add_out(output)

    if motion_filter is not None:
        motion_trig = vmf.create_ent(
            classname='trigger_multiple',
            targetname=conditions.local_name(inst, 'enable_motion_trig'),
            origin=start_pos,
            filtername=motion_filter,
            startDisabled=1,
            wait=0.1,
        )
        motion_trig.add_out(Output('OnStartTouch', '!activator', 'ExitDisabledState'))
        # Match the size of the original...
        motion_trig.solids.append(vmf.make_prism(
            start_pos + Vec(72, -56, 58) @ orient,
            end_pos + Vec(-72, 56, 144) @ orient,
            mat=consts.Tools.TRIGGER,
        ).solid)

    if res.bool('NoPortalFloor'):
        # Block portals on the floor..
        floor_noportal = vmf.create_ent(
            classname='func_noportal_volume',
            origin=track_start,
        )
        floor_noportal.solids.append(vmf.make_prism(
            start_pos + Vec(-60, -60, -66) @ orient,
            end_pos + Vec(60, 60, -60) @ orient,
            mat=consts.Tools.INVISIBLE,
        ).solid)

    # A brush covering under the platform.
    base_trig = vmf.make_prism(
        start_pos + Vec(-64, -64, 48) @ orient,
        end_pos + Vec(64, 64, 56) @ orient,
        mat=consts.Tools.INVISIBLE,
    ).solid

    vmf.add_brush(base_trig)

    # Make a paint_cleanser under the belt..
    if res.bool('PaintFizzler'):
        pfizz = vmf.create_ent(
            classname='trigger_paint_cleanser',
            origin=start_pos,
        )
        pfizz.solids.append(base_trig.copy())
        for face in pfizz.sides():
            face.mat = consts.Tools.TRIGGER

