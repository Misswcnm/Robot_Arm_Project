# ESTUN 机械臂请求字段(给前端)

## 示教(type1)

```json
{ "type": "type1", "mapid": "地图ID", "poseid": "点位ID", "point_type": 1 }
```

| 字段 | 必填 | 说明 |
|---|---|---|
| type | 是 | 固定 `type1` |
| mapid | 是 | 小车地图 ID |
| poseid | 是 | 点位 ID |
| point_type | 是 | `1` = ICP 拖拽示教,`0` = AprilTag 示教 |

> `poseid` 本身已经是 UUID、每个点独立唯一,所以**不再需要 `group_id`**。

## 删除(type3)

```json
{ "type": "type3", "mapid": "地图ID", "poseid": "点位ID" }
```

| 字段 | 必填 | 说明 |
|---|---|---|
| type | 是 | 固定 `type3` |
| mapid | 是 | 小车地图 ID |
| poseid | 是 | 要删除的点位 ID |
| command | 否 | 只删除其中一个点时,额外传 `command` 指定要删的点 |

## ⚠️ 注意

- **AprilTag 示教(`point_type=0`)的 `tag_id` 可选**,不传默认 `0`,传了几就是几:

  ```json
  { "type": "type1", "mapid": "…", "poseid": "…", "point_type": 0 }
  { "type": "type1", "mapid": "…", "poseid": "…", "point_type": 0, "tag_id": 3 }
  ```

- ICP 示教(`point_type=1`)不需要 `tag_id`,只需要 `mapid + poseid + point_type`。
