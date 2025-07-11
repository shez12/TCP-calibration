import cv2
import base64
import re
import json
# from modelscope import Qwen2_5_VLForConditionalGeneration, AutoProcessor
# from qwen_vl_utils import process_vision_info

IMG_FORMAT = '.png'

class ObjectDetect:
    def __init__(self):
        from modelscope import Qwen2_5_VLForConditionalGeneration, AutoProcessor
        from qwen_vl_utils import process_vision_info
        self.process_vision_info = process_vision_info
        self.model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
        "Qwen/Qwen2.5-VL-3B-Instruct", torch_dtype="auto", device_map="auto")
        self.processor = AutoProcessor.from_pretrained("Qwen/Qwen2.5-VL-3B-Instruct")
        # self.model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
        # "Qwen/Qwen2.5-VL-7B-Instruct", torch_dtype="auto", device_map="auto")
        # self.processor = AutoProcessor.from_pretrained("Qwen/Qwen2.5-VL-7B-Instruct")

    def img2base64(self, img:cv2.typing.MatLike) -> str:
        assert img is not None
        success, buffer = cv2.imencode(IMG_FORMAT, img)
        if not success:
            return None
        
        img_base64 = base64.b64encode(buffer).decode("utf-8")
        return img_base64

    def detect(self, object: str, image:cv2.typing.MatLike, max_new_token=128):
        """
        return the bbox list of the object in image
        """
        img_base64 = self.img2base64(image)
        messages = [
            {
                "role": "user",
                "content": [
                    {
                        "type": "image",
                        "image": "data:image;base64,"+img_base64,
                    },
                    {"type": "text", "text": f"找出图片中 {object} 的像素坐标， 并以jason格式返回物体框的像素坐标"},
                ],
            }
        ]
        # Preparation for inference
        text = self.processor.apply_chat_template(
            messages, tokenize=False, add_generation_prompt=True
        )
        image_inputs, video_inputs = self.process_vision_info(messages)
        inputs = self.processor(
            text=[text],
            images=image_inputs,
            videos=video_inputs,
            padding=True,
            return_tensors="pt",
        )
        inputs = inputs.to("cuda")

        # Inference: Generation of the output
        generated_ids = self.model.generate(**inputs, max_new_tokens=max_new_token)
        generated_ids_trimmed = [
            out_ids[len(in_ids) :] for in_ids, out_ids in zip(inputs.input_ids, generated_ids)
        ]
        output_text = self.processor.batch_decode(
            generated_ids_trimmed, skip_special_tokens=True, clean_up_tokenization_spaces=False
        )
        try:
            json_str = re.search(r'```json\n([\s\S]*?)\n```', output_text[0]).group(1)
            data = json.loads(json_str)
            return data[0]["bbox_2d"]
        except:
            print(f"FAILED, cannot find {object} in the image")
            return None


if __name__=="__main__":
    img = cv2.imread('imgs/fruit.png')
    cv2.imshow("img", img)
    cv2.waitKey(2)
    detector = ObjectDetect()
    # result = detector.detect("the eye of the women", img)
    # result = detector.detect("the hand of the women", img)
    result = detector.detect("the orange", img)
    print(result)

    img = cv2.rectangle(img, (result[0], result[1]), (result[2], result[3]), color=(0,0,255), thickness=2)
    cv2.imshow("img", img)
    cv2.waitKey(0)