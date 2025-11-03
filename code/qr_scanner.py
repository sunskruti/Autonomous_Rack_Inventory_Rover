
import cv2, json, time, os
from pyzbar.pyzbar import decode
import warnings
warnings.filterwarnings("ignore")


OUT_DIR = "output_snippets"
os.makedirs(OUT_DIR, exist_ok=True)
results = []

cap = cv2.VideoCapture(0)  
print("Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur_metric = cv2.Laplacian(gray, cv2.CV_64F).var()
    if blur_metric < 50:
        cv2.putText(frame, "Blurred", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2)

   
    frame = cv2.convertScaleAbs(frame, alpha=1.2, beta=20)


    for q in decode(frame):
        data = q.data.decode("utf-8")
        (x,y,w,h) = q.rect
        crop = frame[y:y+h, x:x+w]
        fname = f"{OUT_DIR}/{int(time.time())}.jpg"
        cv2.imwrite(fname, crop)

        results.append({
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "rack_id": "Rack-1",
            "QR_data": data,
            "image_snippet": fname
        })

        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        cv2.putText(frame, data,(x,y-10),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)

    cv2.imshow("QR Scanner", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


json_path = r"C:\Users\sansk\qrscanner\qr_results.json"
with open(json_path,"w") as f:
    json.dump(results, f, indent=2)
print("Saved detections to qr_results.json")


