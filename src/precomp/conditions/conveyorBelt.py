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

    New conveyor belt requires more options since we generate the middle.
    * New Options
        * `New`: Are we generating a new conveyor belt type?
        * `SegmentInst`: The instance for the conveyor belt segments.
        * `TrackInst`: The track the segments ride on.
    """
    new_conveyor: bool = res.bool('New', False)

    if new_conveyor:
        # Generate our new conveyor instead

        inst_name = inst['targetname'].casefold()
        inst_file = inst['file'].casefold()
        
        item = ITEMS[inst_name]

        if not item.outputs:
            # Item has no outputs and is probably an end\
            #LOGGER.info("Conveyor Belt " + inst_name + " has no outputs")
            return

        LOGGER.info("Generating Conveyor Belt " + inst_name)

        start_marker = Marker(inst)
        end_marker_dict: dict[str, Marker] = {}

        for output in item.outputs:
            conn_item = output.to_item
            conn_inst = conn_item.inst
            if conn_inst['file'].casefold() == inst_file:
                if end_marker_dict:
                    raise ValueError(f'Conveyor belt {inst_name} has two connections!')
                end_marker_dict[conn_inst['file']] = Marker(conn_inst)
                for conn_output in conn_item.outputs:
                    if conn_output.to_item.name == item.name:
                        raise ValueError('Cyclical Conveyor Belt connection (ends are connected to eachother)!')
            else:
                raise ValueError(f'Conveyor Belt {inst_name} connected to non-conveyor belt!')

        if not end_marker_dict:
            LOGGER.info(f"Conveyor belt {inst_name} has no end instance")
            return
        
        end_marker = end_marker_dict[inst_file]

        item.delete_antlines()

        start_pos = Vec.from_str(inst['origin'])
        end_pos = Vec.from_str(end_marker.ent['origin'])

        # We need the up norm because they're wall mounted
        start_norm = start_marker.orient.forward()
        end_norm = end_marker.orient.forward()

        size_vec = abs((start_pos + (Vec(64, 0, 0) @ start_norm.to_angle())) - (end_pos + (Vec(64, 0, 0) @ end_norm.to_angle())))
        size: int = int((size_vec.x + size_vec.y + size_vec.z) / 128)
        #LOGGER.info("Belt Size: " + str(size))

        inst.fixup['$size'] = size

        # These checks are incredibly messy, simplify them later
        if start_norm.axis() == 'x':
            if not start_pos.y == end_pos.y and not start_pos.z == end_pos.z:
                raise ValueError(f'Conveyor Belts are not in line (x axis) {start_pos} {end_pos}')
        if start_norm.axis() == 'y':
            if not start_pos.x == end_pos.x and not start_pos.z == end_pos.z:
                raise ValueError(f'Conveyor Belts are not in line (y axis) {start_pos} {end_pos}')
        if start_norm.axis() == 'z':
            if not start_pos.x == end_pos.x and not start_pos.y == end_pos.y:
                raise ValueError(f'Conveyor Belts are not in line (z axis) {start_pos} {end_pos}')
        if not start_norm == -end_norm:
            raise ValueError(f'Conveyor Belts are not facing eachother {start_norm} {end_norm}')

        segment_inst_file = instanceLocs.resolve_one(res['SegmentInst', ''], error=False)
        track_inst_file = instanceLocs.resolve_one(res['TrackInst', ''], error=False)

        offset = 256
        track_name = conditions.local_name(inst, '&segment{}')
        track_start: Vec = start_pos + Vec(offset, 0, 32) @ start_norm.to_angle()
        track_end: Vec = end_pos + (Vec(offset, 0, 32) @ end_norm.to_angle())

        norm = start_marker.orient.up()

        if res.bool('rotateSegments', True):
            orient = Matrix.from_basis(x=start_norm, z=norm)
            inst['angles'] = orient.to_angle()
        else:
            orient = start_marker.orient

        for index, pos in enumerate(track_start.iter_line(track_end, stride=128), start=1):
            # Don't place at the last point - it doesn't teleport correctly,
            # and would be one too many.
            if segment_inst_file and pos != track_end:
                seg_inst = conditions.add_inst(
                    vmf,
                    targetname=track_name.format(index),
                    file=segment_inst_file,
                    origin=start_pos, #spawn these at the same spot so they have the same lighting
                    angles=orient,
                )
                seg_inst.fixup.update(inst.fixup)

        for index, pos in enumerate(start_pos.iter_line(end_pos, stride=128), start=1):
            conditions.add_inst(
                vmf,
                targetname=inst_name + f'-track{index}',
                file=track_inst_file,
                origin=pos,
                angles=start_marker.orient,
            )

        end_marker.ent.remove()

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
