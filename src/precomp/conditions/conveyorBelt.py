"""Continuously moving belts, like in BTS.
"""
from __future__ import annotations

import attrs
from srctools import Keyvalues, Vec, Entity, Output, VMF, Matrix

import srctools.logger

from precomp import instanceLocs, template_brush, conditions, brushLoc
from precomp.connections import ITEMS, Item
import consts
import user_errors

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
        * `EndOutput`: Adds an output to the last track. The value is the same as
          outputs in VMFs.
        * `BeamKeys`: If set, a list of keyvalues to use to generate an env_beam
          travelling from start to end. The origin is treated specially - X is
          the distance from walls, y is the distance to the side, and z is the
          height.
        * `PaintFizzler`: If set, add a paint fizzler underneath the belt.
        * `RemovePaint`: If set, adds an output to the end triggers to remove the
          paint from segments as they pass.

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

        for output in list(mark1.item.outputs)[:]:
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
                output.remove()
            else:
                raise ValueError(f'Conveyor Belt {mark1.name} connected to non-conveyor belt!')

        if mark2 is None:
            LOGGER.info(f"Conveyor belt {mark1} has no end instance")
            return

        mark1.item.delete_antlines()

        for conn in list(mark2.item.inputs)[:]:
            conn.to_item = mark1.item
        
        conn_count = len(mark1.item.inputs)
        no_conn = conn_count == 0 and not inst.fixup.bool('$start_enabled')

        if no_conn:
            inst.fixup['$speed'] = 0

        size: int = round((
            (mark1.pos + mark1.orient.forward(64)) -
            (mark2.pos + mark2.orient.forward(64))
        ).mag() / 128)
        #LOGGER.info("Belt Size: " + str(size))

        inst.fixup['$size'] = size
        inst.fixup['$angle_fixup'] = Matrix.from_angstr(inst['angles']).transpose().to_angle()

        # Check if axis, facing, and up vector match
        marks_dist_between: Vec = mark2.pos - mark1.pos
        marks_vec_between: Vec = marks_dist_between.norm()
        error = ''
        if marks_dist_between.mag() > 1 and not Vec.dot(mark1.orient.forward(), marks_vec_between) < -0.9999:
            error = 'Conveyor Belts are not in line: {} @ {} -> {} @ {}',
        if not Vec.dot(mark1.orient.forward(), mark2.orient.forward()) < -0.9999:
            error = 'Conveyor Belts are not facing eachother: {} @ {} -> {} @ {}'
        if not Vec.dot(mark1.orient.up(), mark2.orient.up()) > 0.9999:
            error = 'Conveyor Belts do not share the same rotation: {} @ {} -> {} @ {}'
        if error:
            # Document the exact condition in the logs, but just use the same error for users -
            # explaining how we caught it is more complex to explain, and it's pretty obvious
            # how they're not lined up.
            LOGGER.error(
                error,
                mark1.pos, mark1.orient.to_angle(),
                mark2.pos, mark2.orient.to_angle(),
            )
            raise user_errors.UserError(user_errors.TOK_CONVEYOR_NOT_LINED_UP, points=[mark1.pos, mark2.pos])
        del error

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

        # Iterate positions, looking for the best place for lighting, and checking validity.
        # All the belts are placed at the same place, so they get uniform lighting
        marks_voxel_side = mark1.orient.left(128)
        marks_voxel_up = mark1.orient.up(128)
        # If we find obstructions, store to show all of them in the error..
        invalid_pos = []
        # First is our obstruction score, then store the distance to the midpoint, in case of ties.
        # Last is the actual position to use.
        potential_lighting_pos: list[tuple[float, float, Vec]] = []
        # If we don't have any good positions, pick the midpoint.
        lighting_pos = (mark1.pos + mark2.pos) / 2
        for pos in mark1.pos.iter_line(mark2.pos, stride=128):
            if brushLoc.POS.lookup_world(pos).is_solid:
                invalid_pos.append(pos)
                continue
            score = 0
            if not brushLoc.POS.lookup_world(pos + marks_voxel_up).is_solid:
                score += 6  # If the top is visible, always prefer that.
            if not brushLoc.POS.lookup_world(pos + marks_voxel_side).is_solid:
                score += 2  # Sides are equally valuable.
            if not brushLoc.POS.lookup_world(pos - marks_voxel_side).is_solid:
                score += 2
            if not brushLoc.POS.lookup_world(pos - marks_voxel_up).is_solid:
                score += 1  # Below being exposed is better than nothing, but not very important.
            potential_lighting_pos.append((score, -(pos - lighting_pos).len_sq(), pos))
        if invalid_pos:
            raise user_errors.UserError(
                user_errors.TOK_CONVEYOR_OBSTRUCTED, # type: ignore
                points=[mark1.pos, mark2.pos],
                voxels=invalid_pos,
            )
        if potential_lighting_pos:
            LOGGER.debug(
                'Conveyor {} -> {} lighting: {}',
                mark1.pos, mark2.pos, potential_lighting_pos,
            )
            # Pick the one with the highest score.
            lighting_pos = max(potential_lighting_pos)[2]

        offset = 256
        track_name = conditions.local_name(inst, 'segment{}')
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
                    origin=lighting_pos,
                    angles=orient,
                )
                seg_inst.fixup.update(inst.fixup)

        for index, pos in enumerate(mark1.pos.iter_line(mark2.pos, stride=128), start=1):
            track_inst = conditions.add_inst(
                vmf,
                targetname=conditions.local_name(inst, f'track{index}'),
                file=track_inst_file,
                origin=pos,
                angles=mark1.orient,
            )
            track_inst.fixup.update(inst.fixup)

        end_filter = vmf.create_ent(
            classname='filter_activator_name',
            targetname=conditions.local_name(inst, 'filter_segment'),
            filtername=conditions.local_name(inst, 'segment*'),
            origin=mark1.pos,
        )

        if end_filter is not None:
            end_trig_start = vmf.create_ent(
                classname='trigger_multiple',
                targetname=conditions.local_name(inst, 'end_trig_start'),
                origin=mark1.pos,
                filtername=end_filter['targetname'],
                spawnflags=64,
                startDisabled=1,
                wait=0.1,
            )

            end_trig_start.solids.append(vmf.make_prism(
                mark1.pos + Vec(296, -60, -60) @ mark1.orient,
                mark1.pos + Vec(312, 60, 60) @ mark1.orient,
                mat=consts.Tools.TRIGGER,
            ).solid)

            end_trig_end = vmf.create_ent(
                classname='trigger_multiple',
                targetname=conditions.local_name(inst, 'end_trig_end'),
                origin=mark2.pos,
                filtername=end_filter['targetname'],
                spawnflags=64,
                startDisabled=1,
                wait=0.1,
            )

            end_trig_end.solids.append(vmf.make_prism(
                mark2.pos + Vec(296, -60, -60) @ mark2.orient,
                mark2.pos + Vec(312, 60, 60) @ mark2.orient,
                mat=consts.Tools.TRIGGER,
            ).solid)

            for prop in res.find_all('EndOutput'):
                output = Output.parse(prop)
                output.output = 'OnTrigger'
                output.inst_out = None
                output.comma_sep = False
                output.target = conditions.local_name(inst, output.target)
                end_trig_start.add_out(output)
                end_trig_end.add_out(output)

            remove_paint = res['RemovePaint', None]
            if remove_paint is None:
                remove_paint = inst.fixup.bool('$disable_autorespawn', False)
            else:
                remove_paint = res.bool('RemovePaint')
                
            if remove_paint:
                remove_paint_output = Output('OnTrigger', '!activator', 'RemovePaint')
                end_trig_start.add_out(remove_paint_output)
                end_trig_end.add_out(remove_paint_output)

        # Add the EnableMotion trigger_multiple seen in platform items.
        # This wakes up cubes when it starts moving.
        motion_filter = res['MotionTrig', None]
        push_speed = res['speed', None]
        if push_speed is None:
            push_speed = inst.fixup.float('$speed')

        # Disable on walls, or if the conveyor can't be turned on.
        if norm != (0, 0, 1) or no_conn:
            motion_filter = None
            push_speed = None
        
        if motion_filter is not None:
            motion_trig = vmf.create_ent(
                classname='trigger_multiple',
                targetname=conditions.local_name(inst, 'enable_motion_trig'),
                origin=mark1.pos,
                filtername=motion_filter,
                spawnflags=8,
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

        if push_speed is not None:
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
            beam['origin'] = mark1.pos + Vec(
                beam_off.x, beam_off.y, beam_off.z,
            ) @ mark1.orient
            beam['TargetPoint'] = mark2.pos + Vec(
                beam_off.x, beam_off.y, beam_off.z,
            ) @ mark2.orient

        mark2.ent.remove()

        LOGGER.debug(
            f"""
            Options:
            - New: {res['New', '']}
            - EndInst: {res['EndInst', '']}
            - SegmentInst: {res['SegmentInst', '']}
            - TrackInst: {res['TrackInst', '']}
            - Speed: {res['Speed', '']}
            - MotionTrig: {res['MotionTrig', '']}
            - EndOutput: {res['EndOutput', '']}
            - BeamKeys: {res['BeamKeys', '']}
            - PaintFizzler: {res['PaintFizzler', '']}
            - RemovePaint: {res['RemovePaint', '']}

            Instvars:
            - Type: {inst.fixup['$cube_type']} 
            - Start enabled: {inst.fixup['$start_enabled']} 
            - Start reversed: {inst.fixup['$start_reversed']} 
            - Start active: {inst.fixup['$start_active']} 
            - Auto-respawn: {inst.fixup['$disable_autorespawn']}

            - Connection Count: {conn_count}
            - Size: {inst.fixup['$size']}
            - Speed {inst.fixup['$speed']}

            - Sound Move: {inst.fixup['$sound_move']}
            - Sound Start: {inst.fixup['$sound_start']}
            - Sound Reverse: {inst.fixup['$sound_reverse']}
            - Sound Stop: {inst.fixup['$sound_stop']}
            """
        )
            
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
