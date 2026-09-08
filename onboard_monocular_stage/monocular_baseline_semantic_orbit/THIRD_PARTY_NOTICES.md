# 来源与修改

估计器来自当前工程 `approach_demo/relative_estimator.py`，匹配器选取原 `closeskill/template_target_approach.py` 的五个匹配类/函数；来源哈希记录在 `SOURCE_MANIFEST.json`。

SuperPoint、LightGlue 和工具函数来自本机原项目的 `closeskill/lightglue/`。原文件头保留；此打包不重新授予第三方代码或模型许可。当前本机副本未附独立 LICENSE 文件，SuperPoint 文件头含原始 Magic Leap 权利声明，使用或再分发应遵循原权利人的适用条款。

对第三方文件的改动仅为：裁剪包入口导入到 LightGlue/SuperPoint；将这两份模型的权重下载替换成本地 `torch.load(..., weights_only=True)`。没有修改网络结构或模型权重。
