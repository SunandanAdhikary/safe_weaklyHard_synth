import os
import json


def load_input_cfg(root_path=None, print_cfg=True):
    """Load input_cfg.json from project root (or given root_path).

    Returns a dict with keys: fixed_step, rem, cutoff, prec, orders
    """
    if root_path is None:
        root_path = os.getcwd()
    cfg_path = os.path.join(root_path, 'input_cfg.json')
    cfg = {}
    if os.path.exists(cfg_path):
        with open(cfg_path, 'r') as f:
            try:
                cfg = json.load(f)
            except Exception:
                cfg = {}

    hyperperiod = cfg.get('hyperpeiod', 8)
    fixed_step = cfg.get('fixed_step', 0.00005)
    rem = cfg.get('rem', 1e-3)
    cutoff = cfg.get('cutoff', 1e-8)
    prec = cfg.get('prec', 53)

    min_order = cfg.get('min_order', cfg.get('orders_min', None))
    max_order = cfg.get('max_order', cfg.get('orders_max', None))
    if min_order is not None and max_order is not None:
        orders = [int(min_order), int(max_order)]
    else:
        orders = cfg.get('orders', [3, 5])

    run_multicore = cfg.get('run_multicore', 0)
    which_safety = cfg.get('which_safety', 0)
    system_dir = cfg.get('system_dir', os.getcwd())

    if print_cfg:
        print(f"config params:\n hyperperiod={hyperperiod}, fixed_step={fixed_step},\n rem={rem}, cutoff={cutoff},\n prec={prec}, orders={orders},\n run_multicore={run_multicore}, which_safety={which_safety}, \n system_dir={system_dir}")
    return {
        'hyperperiod': hyperperiod,
        'fixed_step': fixed_step,
        'rem': rem,
        'cutoff': cutoff,
        'prec': prec,
        'orders': orders,
        'run_multicore': run_multicore,
        'which_safety': which_safety,
        'system_dir': system_dir
    }
