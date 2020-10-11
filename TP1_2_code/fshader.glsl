#version 150

uniform sampler2D textureGrass;
uniform sampler2D textureRock;
uniform sampler2D textureSnow;


in vec2 v_texcoord;
in vec3 v_position;
//! [0]
void main()
{
    vec2 rescaled_texcoord = v_texcoord * 30;
    // Set fragment color from texture

    if(v_position.y > 0.5)
        gl_FragColor = texture2D(textureSnow, rescaled_texcoord);
    else if(v_position.y > 0.4){
        float altitude = (v_position.y -0.4)*10;
        gl_FragColor = altitude * texture2D(textureSnow, rescaled_texcoord) + (1-altitude)*texture2D(textureRock, rescaled_texcoord);
    }
    else if(v_position.y > 0.3)
        gl_FragColor = texture2D(textureRock, rescaled_texcoord);
    else if(v_position.y > 0.2){
        float altitude = (v_position.y -0.2)*10;
        gl_FragColor = altitude * texture2D(textureRock, rescaled_texcoord) + (1-altitude)*texture2D(textureGrass, rescaled_texcoord);
    }
    else
        gl_FragColor = texture2D(textureGrass, rescaled_texcoord);

}
//! [0]

