from srctools import Entity
import srctools.logger
from hammeraddons.bsp_transform import Context, trans

LOGGER = srctools.logger.get_logger(__name__)


@trans('BEE2: Coop Responses')
def generate_coop_responses(ctx: Context) -> None:
    """Add the coop response script."""
    responses: dict[str, list[str]] = {}
    for response in ctx.vmf.by_class['bee2_coop_response']:
        responses[response['type'].casefold()] = [
            value for key, value in response.items()
            if key.startswith('choreo')
        ]
        response.remove()

    # Always add even if no responses are present and we're in MP - this is also
    # used for general coop-death callbacks.
    if ctx.vmf.spawn['BEE2_game_mode'].casefold() == 'sp':
        if responses:
            LOGGER.warning(
                "bee2_coop_response entities present, but we're in singleplayer. "
                "Still adding script.")
        else:  # Singleplayer and no responses, skip.
            return

    script = ["BEE2_RESPONSES <- {"]
    for response_type, lines in sorted(responses.items()):
        script.append(f'\t{response_type} = [')
        for line in lines:
            script.append(f'\t\tCreateSceneEntity("{line}"),')
        script.append('\t],')
    script.append('};')

    # We want to write this onto the '@glados' entity.
    ent: Entity | None = None
    for ent in ctx.vmf.by_target['@glados']:
        ctx.add_code(ent, '\n'.join(script))
        # Also include the actual script.
        split_script = ent['vscripts'].split()
        split_script.append('bee2/coop_responses.nut')
        ent['vscripts'] = ' '.join(split_script)

    if ent is None and responses:
        LOGGER.warning('Response scripts present, but @glados is not!')
