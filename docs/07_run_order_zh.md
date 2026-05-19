# 推荐复现顺序

> 下面命令需要根据你本机的 SO-101 baseline 资源路径调整脚本中的项目根目录。

## 1. baseline 检查

```bash
python src/baseline_validation/inspect_so101_urdf.py
python src/baseline_validation/random_motion_so101.py
python src/baseline_validation/generate_structure_breakdown.py
```

## 2. 工作空间、力矩和结构瓶颈

```bash
python src/structural_analysis/baseline_workspace_analysis.py
python src/structural_analysis/baseline_static_torque_analysis.py
python src/structural_analysis/baseline_structural_bottleneck_analysis.py
```

## 3. upper arm CAD 生成与校核

```bash
python src/cad/upper_arm_v1/upper_arm_v1_cad.py
python src/cad/upper_arm_v2/upper_arm_v2_cad.py
python src/cad/upper_arm_v3/upper_arm_v3_cad.py
```

随后运行对应版本的 `validate_*`、`mounting_check_*`、`standard_part_assembly_check_*` 和 `urdf_smoke_*` 脚本。

## 4. FEA screening

```bash
python src/fea/run_v3_cae_geometry_cleanup.py
python src/fea/run_full_v3_clean_fea.py
python src/fea/run_calculix_smoke_fea.py
```

## 5. 图表生成

```bash
python src/visualization/generate_upper_arm_model_visuals.py
python src/visualization/generate_simulation_fea_visual_package.py
```
