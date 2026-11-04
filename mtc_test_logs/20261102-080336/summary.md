| 案例 | 參數 | 預期 | 規劃 | 規劃耗時（秒） | 執行 |
| --- | --- | --- | --- | --- | --- |
| m2_default | `預設` | 預期成功（預設參數，已手動驗證過） | ok | 1.27 | ok |
| m3a_object_near | `object_x:=0.4 object_y:=-0.2` | 不確定，觀察 | ok | 1.33 | ok |
| m3b_object_far | `object_x:=1.0 object_y:=-0.25` | 預期規劃失敗（物體超出手臂可達範圍） | fail | 1.30 | skipped |
| m4a_place_y_0.3 | `place_y:=0.3` | 不確定，觀察 | ok | 1.38 | ok |
| m4b_place_y_1.0 | `place_y:=1.0` | 預期規劃失敗（放置點超出可達範圍） | fail | 56.44 | skipped |
