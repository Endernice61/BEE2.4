"""Logic for trigger items, allowing them to be resized."""
from __future__ import annotations

from contextlib import suppress
import copy

from srctools import Keyvalues, Vec, Output, VMF, Entity
import srctools.logger
import attrs

from precomp import instanceLocs, connections, options, conditions
import consts
import utils


COND_MOD_NAME: str | None = None
LOGGER = srctools.logger.get_logger(__name__, alias='cond.resizeTrig')


@attrs.frozen(kw_only=True)
class Brush:
    """A brush which can be placed."""
    material: str
    keys: Keyvalues
    local_keys: Keyvalues
    coop_playerteam: bool

    @classmethod
    def parse(cls, kv: Keyvalues) -> Brush:
        """Parse from the provided keyvalues."""
        return cls(
            material=kv['material', consts.Tools.TRIGGER],
            keys=kv.find_block('keys', or_blank=True),
            local_keys=kv.find_block('localkeys', or_blank=True),
            coop_playerteam=kv.bool('coop_playerteam', True),
        )


@conditions.make_result('ResizeableTrigger', valid_before=conditions.MetaCond.Connections)
def res_resizeable_trigger(vmf: VMF, info: conditions.MapInfo, res: Keyvalues) -> object:
    """Replace two markers with one or more trigger brushes.

    This is run once to affect all of an item.
    Options:

    * `markerInst`: <ITEM_ID:1,2> value referencing the marker instances, or a filename.
    * `markerItem`: The item's ID
    * `previewConf`: A item config which enables/disables the preview overlay.
    * `previewInst`: An instance to place at the marker location in preview mode.
        This should contain checkmarks to display the value when testing.
    * `previewMat`: If set, the material to use for an overlay func_brush.
        The brush will be parented to the trigger, so it vanishes once killed.
        It is also non-solid.
    * `previewScale`: The scale for the func_brush materials.
    * `previewActivate`, `previewDeactivate`: The VMF output to turn the
        previewInst on and off.
    * `triggerActivate, triggerDeactivate`: The `instance:name;Output`
        outputs used when the trigger turns on or off.
    * `coopVar`: The instance variable which enables detecting both Coop players.
        The trigger will be a trigger_playerteam.
    * `coopActivate, coopDeactivate`: The `instance:name;Output` outputs used
        when coopVar is enabled. These should be suitable for a logic_coop_manager.
    * `coopOnce`: If true, kill the manager after it first activates.
    * `brush`: Each of these creates a brush.
        For a single brush, these options can also be inlined into the main options block.
        * `material`: Material applied to the sides. Defaults to `tools/toolstrigger`.
        * `keys`: A block of keyvalues for the trigger brush. Origin and targetname
            will be set automatically.
        * `coop_playerteam`: If true (default), the brush will be swapped to trigger_playerteam
           if coopVar is enabled.
        * `localkeys`: The same as above, except values will be changed to use
            instance-local names.
    """
    marker = instanceLocs.resolve_filter(res['markerInst'])

    marker_names = set()

    for search_inst in vmf.by_class['func_instance']:
        if search_inst['file'].casefold() in marker:
            marker_names.add(search_inst['targetname'])
            # Unconditionally delete from the map, so it doesn't
            # appear even if placed wrongly.
            search_inst.remove()

    if not marker_names:  # No markers in the map - abort, don't bother parsing configs
        return conditions.RES_EXHAUSTED

    item_id = utils.obj_id(res['markerItem'])

    # Synthesise the connection config used for the final trigger.
    conn_conf_sp = copy.deepcopy(connections.ITEM_TYPES[item_id])
    conn_conf_sp.id += ':trigger'
    conn_conf_sp.output_act = Output.parse_name(res['triggerActivate', 'OnStartTouchAll'])
    conn_conf_sp.output_deact = Output.parse_name(res['triggerDeactivate', 'OnEndTouchAll'])

    # For Coop, we add a logic_coop_manager in the mix so both players can
    # be handled.
    coop_var: str | None
    try:
        coop_var = res['coopVar']
    except LookupError:
        coop_var = conn_conf_coop = None
        coop_only_once = False
    else:
        coop_only_once = res.bool('coopOnce')
        conn_conf_coop = copy.deepcopy(connections.ITEM_TYPES[item_id])
        conn_conf_coop.id += ':trigger'
        conn_conf_coop.output_act = Output.parse_name(res['coopActivate', 'OnChangeToAllTrue'])
        conn_conf_coop.output_deact = Output.parse_name(res['coopDeactivate', 'OnChangeToAnyFalse'])

    # Display preview overlays if it's preview mode, and the config is true
    pre_act: Output | None = None
    pre_deact: Output | None = None
    if not info.is_publishing and options.get_itemconf(res['previewConf', ''], False):
        preview_mat = res['previewMat', '']
        preview_inst_file = res['previewInst', '']
        preview_scale = res.float('previewScale', 0.25)
        # None if not found.
        with suppress(LookupError):
            pre_act = Output.parse(res.find_key('previewActivate'))
        with suppress(LookupError):
            pre_deact = Output.parse(res.find_key('previewDeactivate'))
    else:
        # Deactivate the preview_ options when publishing.
        preview_mat = preview_inst_file = ''
        preview_scale = 0.25

    keep_inst = res.bool('keepInst', False)

    if 'brush' in res:
        brush_confs = [Brush.parse(block) for block in res.find_all('brush')]
    else:
        brush_confs = [Brush.parse(res)]

    # Now go through each brush.
    # We do while + pop to allow removing both names each loop through.
    todo_names = set(marker_names)
    while todo_names:
        targ = todo_names.pop()

        mark1 = connections.ITEMS.pop(targ)
        for conn in mark1.outputs:
            if conn.to_item.name in marker_names:
                mark2 = conn.to_item
                conn.remove()  # Delete this connection.
                todo_names.discard(mark2.name)
                del connections.ITEMS[mark2.name]
                break
        else:
            if any(conn.from_item.name in marker_names for conn in mark1.inputs):
                # It's a marker with an input, the other in the pair
                # will handle everything.
                # But reinstate it in ITEMS.
                connections.ITEMS[targ] = mark1
                continue
            else:
                # If the item doesn't have any marker connections, 'connect'
                # it to itself so we'll generate a 1-block trigger.
                mark2 = mark1

        inst1 = mark1.inst
        inst2 = mark2.inst

        is_coop = coop_var is not None and info.is_coop and (
            inst1.fixup.bool(coop_var) or
            inst2.fixup.bool(coop_var)
        )

        bbox_min, bbox_max = Vec.bbox(
            Vec.from_str(inst1['origin']),
            Vec.from_str(inst2['origin'])
        )
        origin = (bbox_max + bbox_min) / 2

        # Extend to the edge of the blocks.
        bbox_min -= 64
        bbox_max += 64

        manager: Entity | None = None
        pre_inst: Entity | None = None
        brushes = []

        if preview_inst_file:
            pre_inst = conditions.add_inst(
                vmf,
                targetname=targ + '_preview',
                file=preview_inst_file,
                # Put it at the second marker, since that's usually
                # closest to antlines if present.
                origin=inst2['origin'],
            )
            pre_inst.fixup.update(inst1.fixup)

        if is_coop:
            assert coop_var is not None and conn_conf_coop is not None

            manager = vmf.create_ent(
                classname='logic_coop_manager',
                targetname=conditions.local_name(inst1, 'man'),
                origin=origin,
            )

            item = connections.Item(
                mark1.inst,
                conn_conf_coop,
                ind_style=mark1.ind_style,
            )

            if coop_only_once:
                # Kill all the ents when both players are present.
                manager.add_out(
                    Output('OnChangeToAllTrue', manager, 'Kill'),
                    Output('OnChangeToAllTrue', targ, 'Kill'),
                )
        else:
            item = connections.Item(
                mark1.inst,
                conn_conf_sp,
                ind_style=mark1.ind_style,
            )

        for brush_conf in brush_confs:
            trig_ent = vmf.create_ent(
                classname='trigger_multiple',  # Default
                targetname=targ,
                origin=options.GLOBAL_ENTS_LOC(),
                angles='0 0 0',
            )
            trig_ent.solids = [
                vmf.make_prism(
                    bbox_min,
                    bbox_max,
                    mat=brush_conf.material,
                ).solid,
            ]
            brushes.append(trig_ent)

            for child in brush_conf.keys:
                trig_ent[child.real_name] = inst1.fixup.substitute(child.value, allow_invert=True)
            for child in brush_conf.local_keys:
                trig_ent[child.real_name] = conditions.local_name(
                    inst1, inst1.fixup.substitute(child.value, allow_invert=True)
                )

            if manager is not None and brush_conf.coop_playerteam:
                trig_ent['spawnflags'] = '1'  # Clients
                trig_ent['classname'] = 'trigger_playerteam'

                trig_ent.add_out(
                    Output('OnStartTouchBluePlayer', manager, 'SetStateATrue'),
                    Output('OnStartTouchOrangePlayer', manager, 'SetStateBTrue'),
                    Output('OnEndTouchBluePlayer', manager, 'SetStateAFalse'),
                    Output('OnEndTouchOrangePlayer', manager, 'SetStateBFalse'),
                )

            if pre_act is not None and (out_act := item.output_act()) is not None:
                assert pre_inst is not None
                out = pre_act.copy()
                out.inst_out, out.output = out_act
                out.target = conditions.local_name(pre_inst, out.target)
                trig_ent.add_out(out)
            if pre_deact is not None and (out_deact := item.output_deact()) is not None:
                assert pre_inst is not None
                out = pre_deact.copy()
                out.inst_out, out.output = out_deact
                out.target = conditions.local_name(pre_inst, out.target)
                trig_ent.add_out(out)

        # Register, and copy over all the antlines.
        connections.ITEMS[item.name] = item
        mark1.transfer_antlines(item)
        if mark1.outputs:
            mark1.transfer_antlines(item)
        else:
            mark1.delete_antlines()
        if mark2.outputs:
            mark2.transfer_antlines(item)
        else:
            mark2.delete_antlines()

        if preview_mat:
            preview_brush = vmf.create_ent(
                classname='func_brush',
                parentname=targ,
                origin=origin,

                Solidity='1',  # Not solid
                drawinfastreflection='1',  # Draw in goo..

                # Disable shadows and lighting..
                disableflashlight='1',
                disablereceiveshadows='1',
                disableshadowdepth='1',
                disableshadows='1',
            )
            preview_brush.solids = [
                # Make it slightly smaller, so it doesn't z-fight with surfaces.
                vmf.make_prism(
                    bbox_min + 0.5,
                    bbox_max - 0.5,
                    mat=preview_mat,
                ).solid,
            ]
            for face in preview_brush.sides():
                face.scale = preview_scale

        for conn in mark1.outputs | mark2.outputs:
            conn.from_item = item
        for conn in mark1.inputs | mark2.inputs:
            conn.to_item = item

    return conditions.RES_EXHAUSTED
