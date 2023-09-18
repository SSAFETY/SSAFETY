# coding: utf-8
import copy
import datetime
import json
import logging
import math
import os
import os.path as osp

import pytz
from colorlog import ColoredFormatter
from PIL import Image, ImageDraw, ImageFont
from tqdm import tqdm


def merge_lane_annotation(annot_list, save_path):
    images = []
    annotations = []
    annot_id = 1
    image_id = 0
    for annot in tqdm(annot_list):
        with open(annot, "r") as f:
            json_data = json.load(f)
        object_data = json_data["data_set_info"]["data"]
        img_path = annot.replace("ANNOTATION", "IMAGE").replace("json", "jpg")
        if not os.path.exists(img_path):
            continue
        img = Image.open(img_path)

        img_info = {}
        img_info["id"] = image_id
        file_name = osp.basename(img_path)
        img_info["file_name"] = copy.deepcopy(file_name)
        img_info["height"] = copy.deepcopy(img.size[1])
        img_info["width"] = copy.deepcopy(img.size[0])
        images.append(copy.deepcopy(img_info))
        img.close()

        # annotation은 segmentation / iscrowd, image_id, \
        # category_id, id, bbox, area
        for target in object_data:
            obj_label = target["value"]["object_Label"]
            if "lane_type" in obj_label.keys():
                category = obj_label["lane_type"]
            else:
                continue
            ann = {}
            if category in [
                "lane_blue",
                "lane_shoulder",
                "lane_white",
                "lane_yellow",
            ]:
                points = target["value"]["points"]
                temp_points = copy.deepcopy(points)
                # deepcopy를 해야 둘다 변경되지 않음
                area = get_area(temp_points)
                if category == "lane_blue":
                    ann["category_id"] = 1
                elif category == "lane_shoulder":
                    ann["category_id"] = 2
                elif category == "lane_white":
                    ann["category_id"] = 3
                elif category == "lane_yellow":
                    ann["category_id"] = 4
                segmentation = []
                seg_x = []
                seg_y = []
                for point in points:
                    segmentation.append(point["x"])
                    seg_x.append(point["x"])
                    segmentation.append(point["y"])
                    seg_y.append(point["y"])
                bbox = [
                    min(seg_x),
                    min(seg_y),
                    max(seg_x) - min(seg_x),
                    max(seg_y) - min(seg_y),
                ]
                ann["bbox"] = bbox
                ann["segmentation"] = [segmentation]
                ann["area"] = area
                ann["image_id"] = image_id
                ann["iscrowd"] = 0
                ann["id"] = annot_id
                annot_id += 1
            if ann:
                annotations.append(ann)
        image_id += 1
    merged_data = {
        "images": images,
        "annotations": annotations,
        "categories": [
            {"supercategory": "lane", "id": 1, "name": "lane_blue"},
            {"supercategory": "lane", "id": 2, "name": "lane_shoulder"},
            {"supercategory": "lane", "id": 3, "name": "lane_white"},
            {"supercategory": "lane", "id": 4, "name": "lane_yellow"},
        ],
    }
    with open(save_path, "w", encoding="utf-8") as f:
        json.dump(merged_data, f, ensure_ascii=False, indent="\t")


def merge_vehicle_annotation(annot_list, save_path):
    images = []
    annotations = []
    annot_id = 1
    image_id = 0
    for annot in tqdm(annot_list):
        with open(annot, "r") as f:
            json_data = json.load(f)
        object_data = json_data["data_set_info"]["data"]
        img_path = annot.replace("ANNOTATION", "IMAGE").replace("json", "jpg")
        if not os.path.exists(img_path):
            continue
        img = Image.open(img_path)

        img_info = {}
        img_info["id"] = image_id
        file_name = osp.basename(img_path)
        img_info["file_name"] = copy.deepcopy(file_name)
        img_info["height"] = copy.deepcopy(img.size[1])
        img_info["width"] = copy.deepcopy(img.size[0])
        images.append(copy.deepcopy(img_info))
        img.close()

        # annotation은 segmentation / iscrowd, image_id, \
        # category_id, id, bbox, area
        for target in object_data:
            obj_label = target["value"]["object_Label"]
            if "vehicle_type" in obj_label.keys():
                category = obj_label["vehicle_type"]
            else:
                continue
            ann = {}
            if category in [
                "vehicle_car",
                "vehicle_bike",
                "vehicle_bus",
                "vehicle_truck",
            ]:
                points = target["value"]["points"]
                temp_points = copy.deepcopy(points)
                # deepcopy를 해야 둘다 변경되지 않음
                area = get_area(temp_points)
                if category == "vehicle_car":
                    ann["category_id"] = 1
                elif category == "vehicle_bike":
                    ann["category_id"] = 2
                elif category == "vehicle_bus":
                    ann["category_id"] = 3
                elif category == "vehicle_truck":
                    ann["category_id"] = 4
                segmentation = []
                seg_x = []
                seg_y = []
                for point in points:
                    segmentation.append(point["x"])
                    seg_x.append(point["x"])
                    segmentation.append(point["y"])
                    seg_y.append(point["y"])
                bbox = [
                    min(seg_x),
                    min(seg_y),
                    max(seg_x) - min(seg_x),
                    max(seg_y) - min(seg_y),
                ]
                ann["bbox"] = bbox
                ann["segmentation"] = [segmentation]
                ann["area"] = area
                ann["image_id"] = image_id
                ann["iscrowd"] = 0
                ann["id"] = annot_id
                annot_id += 1
            if ann:
                annotations.append(ann)
        image_id += 1
    merged_data = {
        "images": images,
        "annotations": annotations,
        "categories": [
            {"supercategory": "vehicle", "id": 1, "name": "vehicle_car"},
            {"supercategory": "vehicle", "id": 2, "name": "vehicle_bike"},
            {"supercategory": "vehicle", "id": 3, "name": "vehicle_bus"},
            {"supercategory": "vehicle", "id": 4, "name": "vehicle_truck"},
        ],
    }
    with open(save_path, "w", encoding="utf-8") as f:
        json.dump(merged_data, f, ensure_ascii=False, indent="\t")


class CustomFormatter(ColoredFormatter):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

    def converter(self, timestamp):
        # Create datetime in UTC
        dt = datetime.datetime.fromtimestamp(timestamp, tz=pytz.UTC)
        # Change datetime's timezone
        return dt.astimezone(pytz.timezone("Asia/Seoul"))

    def formatTime(self, record, datefmt=None):
        dt = self.converter(record.created)
        if datefmt:
            s = dt.strftime(datefmt)
        else:
            try:
                s = dt.isoformat(timespec="milliseconds")
            except TypeError:
                s = dt.isoformat()
        return s


def setup_logger(
    log_file_path: str = None,
    name: str = "Violation_CLS_MODEL",
    mode: str = "a",
    level=logging.INFO,
):

    logging.basicConfig(
        format="%(asctime)s %(levelname)-8s %(filename)s"
        "[line:%(lineno)d]: %(message)s",
        # Define the format of the output log
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    """Return a logger with a default ColoredFormatter."""
    formatter = CustomFormatter(
        "%(asctime)s %(log_color)s%(levelname)-8s %(reset)s %(filename)s"
        "[line:%(lineno)d]: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        reset=True,
        log_colors={
            "DEBUG": "blue",
            "INFO": "green",
            "WARNING": "yellow",
            "ERROR": "red",
            "CRITICAL": "red",
        },
    )

    logger = logging.getLogger(name)
    if len(logger.handlers) == 0:
        handler = logging.StreamHandler()
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        if log_file_path is not None:
            if not os.path.exists(os.path.dirname(log_file_path)):
                os.makedirs(os.path.dirname(log_file_path))
            file_handler = logging.FileHandler(log_file_path, mode=mode)
            file_handler.setFormatter(formatter)
            logger.addHandler(file_handler)
    logger.setLevel(level)
    logger.propagate = False
    # logger.info('logger init finished')

    return logger


def viz_inference_result(image_path, image_info):
    image = Image.fromarray(image_path)
    # image = Image.open(image_path)
    draw = ImageDraw.Draw(image)

    for obj in image_info.objects:
        if obj.obj_type == "lane":
            continue
        if obj.label == "violation":
            color = (255, 0, 0)
        elif obj.label == "danger":
            color = (255, 178, 102)
        else:
            color = (0, 255, 0)

        draw.rectangle(obj.bbox, outline=color, width=2)
        font = ImageFont.truetype("./fonts/NanumGothic-Bold.ttf", 30)
        text_bbox = draw.textbbox(obj.bbox[:2], obj.label, font=font, stroke_width=3)
        draw.rectangle(text_bbox, fill=color)
        draw.text(obj.bbox[:2], obj.label, font=font, fill=(255, 255, 255))

    return image


def get_area(points):
    append_dict = {"x": points[0]["x"], "y": points[0]["y"]}
    points.append(append_dict)
    plus = 0
    minus = 0
    for i in range(len(points) - 1):
        plus += points[i]["x"] * points[i + 1]["y"]
        minus += points[i]["y"] * points[i + 1]["x"]
    area = math.fabs(0.5 * (plus - minus))

    return area


def create_json(data_id, image_info):
    json_data = {"dataID": data_id}
    file_base = os.path.basename(image_info.image_id)
    data_set_info = {"sourceValue": file_base}

    img_id = file_base.split(".")[0]
    v_type = re.search(r"\[(.*?)\]", img_id).group(0)
    metainfo = img_id.replace(v_type, "")
    metainfo = metainfo.split("_")
    v_type = v_type.replace("[", "").replace("]", "")
    metainfo = {
        "violation_type": v_type,
        "video_id": metainfo[0][:5],
        "camera_channel": metainfo[0][5],
        "time_info": metainfo[1],
        "camera_number": metainfo[-1],
    }

    data = []
    for object_id, obj in enumerate(image_info.objects):
        objectID = f"data_set_info_{data_id}_{object_id + 1}"
        value = {"metainfo": metainfo, "annotation": "POLYGONS"}

        points = []
        for i in range(len(obj.poly) // 2):
            points.append({"x": obj.poly[2 * i], "y": obj.poly[2 * i + 1]})
        value["points"] = points

        if obj.obj_type == "lane":
            extra = {"value": "lane", "label": "차선", "color": "#e0182d"}
            ctg = obj.category
            ctg = ctg.split("_")
            obj_type = f"lane_{ctg[0]}"
            obj_attr = "_".join(ctg[1:])
            object_Label = {"type": obj_type, "attribute": obj_attr}
        else:
            extra = {"value": "car", "label": "차량", "color": "#096ecd"}
            object_Label = {"type": obj.category, "attribute": obj.label}
        value["extra"] = extra
        value["object_Label"] = object_Label
        data.append({"objectID": objectID, "value": value})

    data_set_info["data"] = data
    json_data["data_set_info"] = data_set_info

    return json_data
