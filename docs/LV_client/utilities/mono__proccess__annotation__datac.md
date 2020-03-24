## mono_proccess_annotation_data.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/utilities/mono__proccess__annotation__datac.png" 
width="400"  />
</p>

### Description 
Tool to draw bounding boxes on top an image based on the annotation data provided.

### Inputs
- **Image_in:** The output from the camera.
- **Annotation_data:** The information to draw the bounding box around the classified objects and tags.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **Image_out :** The output image with the bounding box drawn based on the annotation data.
- **Annotation data cluster :** A cluster with the bounding box information and tags.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
