uniform sampler2D tex0;
uniform float phi_range;

varying vec3 local;

void main() {
     float PI = 3.1415926535897932384626433832795;

     theta = - atan(local.y, local.x);
     if (local.x < 0.0) {
        theta += PI;
     }
     if (theta < 0.0) {
        theta += 2 * PI;
     }
     vec2 uv;
     uv[0] = theta / (2 * PI);
     
     float depth = sqrt(local.x * local.x + local.y * local.y);
     float phi = atan(local.z, depth);
     float pixel_offset_from_center = phi / phi_per_pixel;
     uv[1] = max(0.0, min(1.0, 0.5 - pixel_offset_from_center
     uv[1] = max(0.0, min(height - 1.1, height / 2.0 - pixel_offset_from_center));

     gl_FragColor = texture2D(tex0, uv);
}
