# Safe Weakly hard Specification Synthesis

A novel synthesis framework is proposed for synthesising such safe weakly hard specifications given a hybrid system model along with its desired performance and safety boundaries. It internally uses [Flow star](https://github.com/chenxin415/flowstar). The proposed framework offers more scalability and more precise approximation of safety boundaries compared to the state-of-the-art [[1](https://dl.acm.org/doi/10.1145/3302504.3311811)],[[2](https://dl.acm.org/doi/10.1007/978-3-030-53288-8_26)],[[3](https://eskang.github.io/assets/papers/rv20b.pdf#page=3.14)].

<img width="1609" height="820" alt="toolflow" src="https://github.com/user-attachments/assets/bfd9a084-00d1-4e8b-b9cf-ec02d1f5797a" />

## How to run
- requires flowstar(included at /flowstar-2.1.0, need to make build, runs in linux) 
- requires python, wsl, pip (install them)
- run pip install numpy polytope matplotlib 
- modify [input_cfg.json](/input_cfg.json) at root path
- run [mK_safety_assn.py](/mK_safety_assn.py)
    1. create system directory, model filenames from config file using [load_config.py](/utils/config.py) and [system_desc.py](/system_desc.py)
    2. partition state space into multiple grids as polytopes using [partition_safex.py](/partition_safex.py)
    3. generte model files for reachability analysis of every sscd with [generate_sscds.py](/generate_sscds.py)
    4. verify them sequentially using [verify_model.py](/verify_model.py) and parallelly using [verify_model_parallel.py](/verify_model_parallel.py)
    5. check the output logs using [check_log.py](/utils/check_log.py)
    6. store safe partitions for a maximum miss count ```m_j``` and consecutive miss count ```cm_bar_j``` (based on sscd) in a 2d array ```sg_loc``` and limit ```cm``` for the current ```m_j```
    7. repeat 3,4,5,6 for HA models for ```K_max``` and ```m_j``` (model file generation uses [generate_model.py](/generate_model.py))





