"""Build the KoAdjust SOP with Houdini 22.0.466 hython."""
from pathlib import Path
import tempfile

import hou

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / 'otls/sop_ko.koadjust.1.0.hda'


def wrangle(parent, name, source, code, run_over=0):
    node = parent.createNode('attribwrangle', name)
    node.setInput(0, source)
    node.setParms({'class': run_over, 'snippet': code})
    return node


def menu(name, label, items, labels, default):
    return hou.StringParmTemplate(name, label, 1, default_value=(default,), menu_items=items,
                                  menu_labels=labels, menu_type=hou.menuType.StringReplace)


def build(path):
    obj = hou.node('/obj').createNode('geo')
    main = obj.createNode('subnet', 'koadjust')
    # Provide the construction-time defaults before nodes resolve references.
    for name, value in [('part_source','connectivity'),('part_attribute','name'),('uv_attribute','uv'),('view_hidden',''),('active_stage',''),('view_focus','')]:
        main.addSpareParmTuple(hou.StringParmTemplate(name,name,1,default_value=(value,)))
    validate = wrangle(main, 'validate_parts', main.indirectInputs()[0], '''
string mode=chs("../part_source"), named=chs("../part_attribute"), uv=chs("../uv_attribute");
string reason="";
if (mode=="attribute") {
    int t=primattribtype(0,named);
    if (!(t==0 || t==2) || primattribsize(0,named)!=1)
        reason="Parts require an existing scalar string or integer primitive attribute";
}
if (mode=="uv" && (vertexattribtype(0,uv)!=1 || vertexattribsize(0,uv)<2))
    reason="UV island parts require a vertex UV attribute";
s@__ko_adjust_part_key = mode + (mode=="attribute" ? ":"+named : mode=="uv" ? ":"+uv : "");
s@__ko_adjust_part_error = reason;
''')
    conn = main.createNode('connectivity', 'connected_parts')
    conn.setInput(0, validate)
    conn.setParms({'connecttype':1, 'attribname':'__ko_adjust_part', 'attribtype':1, 'prefix':'Part '})
    islands = wrangle(main, 'uv_islands', validate,
                      (ROOT / 'vex/ko_adjust_uv_parts.vfl').read_text(encoding='utf-8'))
    named = wrangle(main, 'named_parts', validate, '''
string name=chs("../part_attribute");
if (string(detail(0,"__ko_adjust_part_error"))=="") {
    if (primattribtype(0,name)==0) s@__ko_adjust_part=itoa(int(prim(0,name,@primnum)));
    else s@__ko_adjust_part=string(prim(0,name,@primnum));
}
''', 1)
    modes = main.createNode('switch', 'choose_parts')
    for i, source in enumerate((validate, conn, named, islands)):
        modes.setInput(i, source)
    modes.parm('input').setExpression('if(strlen(details("../validate_parts", "__ko_adjust_part_error")), 0, if(strcmp(chs("../part_source"),"attribute")==0, 2, if(strcmp(chs("../part_source"),"uv")==0, 3, 1)))')
    stack = main.createNode('subnet', 'stack')
    stack.setInput(0, modes)
    tail = stack.createNode('output', 'OUT')
    tail.setInput(0, stack.indirectInputs()[0])
    tail.setDisplayFlag(True)
    rewind = main.createNode('object_merge','rewound_geometry')
    rewind.parm('xformtype').set(0)
    rewind.parm('objpath1').setExpression('ifs(strcmp(chs("../active_stage"), "__base")==0, "../choose_parts", strcat("../stack/",chs("../active_stage")))')
    source = main.createNode('switch','view_source')
    source.setInput(0,stack)
    source.setInput(1,rewind)
    source.parm('input').setExpression('strlen(chs("../active_stage")) > 0')
    report = wrangle(main, 'report_unresolved_edits', source, '''
string error = detail(0,"__ko_adjust_part_error");
if (error!="") warning("KoAdjust: %s",error);
string names[] = detailintrinsic(0,"detailattributes");
foreach (string name; names) {
    if (!startswith(name,"ko_adjust_status_")) continue;
    string issue = detail(0,name);
    if (issue != "") warning("KoAdjust edit needs attention: %s",issue);
}''')
    mask = wrangle(main, 'viewport_mask', report, '''
string data=chs("../view_hidden"), focus=chs("../view_focus");
if (data!="" || focus!="") {
    dict hidden=json_loads(data,2), edited=json_loads(focus,2);
    for (int i=0;i<nprimitives(0);i++) {
        string name=prim(0,"__ko_adjust_part",i);
        int hidden_by_user=isvalidindex(hidden,name);
        int ghost=!hidden_by_user && focus!="" && !isvalidindex(edited,name);
        if (hidden_by_user || ghost) setprimgroup(0,"__ko_adjust_hidden",i,1);
        if (ghost) setprimgroup(0,"__ko_adjust_ghost",i,1);
    }
}''')
    ghosts = main.createNode('blast','ghost_geometry')
    ghosts.setInput(0,mask)
    ghosts.setParms({'group':'__ko_adjust_ghost','negate':1})
    before = main.createNode('blast','visible_before')
    before.setInput(0,mask)
    before.parm('group').set('__ko_adjust_hidden')
    visibility = main.createNode('visibility', 'viewport_visibility')
    visibility.setInput(0, mask)
    visibility.setParms({'group':'__ko_adjust_hidden','action':0,'cumulative':1})
    view = main.createNode('switch','show_parts')
    view.setInput(0,report)
    view.setInput(1,visibility)
    view.parm('input').setExpression('strlen(chs("../view_hidden"))+strlen(chs("../view_focus")) > 0')
    clean = main.createNode('attribdelete','clean_private_parts')
    clean.setInput(0,view)
    clean.setParms({'primdel':'__ko_adjust_part','dtldel':'__ko_adjust_part_key __ko_adjust_part_error ko_adjust_status_*'})
    clean_groups = main.createNode('groupdelete','clean_temporary_mask')
    clean_groups.setInput(0,clean)
    clean_groups.parm('group1').set('__ko_adjust_hidden __ko_adjust_ghost')
    out = main.createNode('output', 'OUT')
    out.setInput(0, clean_groups)
    out.setDisplayFlag(True)
    main.layoutChildren()
    node = main.createDigitalAsset(name='ko::koadjust::1.0', hda_file_name=str(path),
                                   description='KoAdjust', min_num_inputs=1, max_num_inputs=1,
                                   compress_contents=True, create_backup=False)
    definition = node.type().definition()
    definition.addSection('EditableNodes', 'stack')
    definition.addSection('MessageNodes', 'report_unresolved_edits')
    definition.addSection('DefaultState', 'ko_adjust')
    for section, name in [('BrushBindVEX','ko_adjust_bind'), ('BrushDeformVEX','ko_adjust_deform')]:
        definition.addSection(section, (ROOT / f'vex/{name}.vfl').read_text(encoding='utf-8'))
    definition.addSection('Help', (ROOT / 'docs/adjust-quickstart.md').read_text(encoding='utf-8'))
    ptg = hou.ParmTemplateGroup()
    ptg.append(menu('tool','Brush',('grab','clay','inflate','smooth','relax','flatten'),('Move','Clay','Inflate','Smooth','Relax','Planar'),'grab'))
    direction=menu('move_direction','Move Direction',('view','normal'),('View Plane','Surface Normal'),'view')
    direction.setConditional(hou.parmCondType.HideWhen,'{ tool != grab }')
    ptg.append(direction)
    ptg.append(menu('influence','Affect',('part','nearby','connected'),('One Part','Nearby in 3D','Topologically Connected'),'part'))
    ptg.append(hou.FloatParmTemplate('radius', 'Radius', 1, default_value=(.25,), min=0., min_is_strict=True))
    parts_folder = hou.FolderParmTemplate('parts_folder','Parts',folder_type=hou.folderType.Simple)
    parts_folder.addParmTemplate(menu('part_source','Define Parts By',('connectivity','attribute','uv'),
                    ('Connectivity','Primitive Attribute','UV Islands'),'connectivity'))
    for name, label, value, mode in [('part_attribute','Part Attribute','name','attribute'),('uv_attribute','UV Attribute','uv','uv')]:
        parm = hou.StringParmTemplate(name,label,1,default_value=(value,))
        parm.setConditional(hou.parmCondType.HideWhen,'{ part_source != '+mode+' }')
        parts_folder.addParmTemplate(parm)
    parts_folder.addParmTemplate(hou.ButtonParmTemplate('open_parts','Open Parts List',
        script_callback='import ko_adjust_parts\nko_adjust_parts.open_panel(kwargs["node"])',
        script_callback_language=hou.scriptLanguage.Python))
    ptg.append(parts_folder)
    ptg.append(hou.StringParmTemplate('active_stage', 'Rewind To', 1,
               item_generator_script="import ko_adjust\nreturn ko_adjust.stage_menu(kwargs)",
               item_generator_script_language=hou.scriptLanguage.Python, menu_type=hou.menuType.StringReplace))
    ptg.append(hou.ToggleParmTemplate('wire', 'Show Before Wireframe', default_value=True))
    upgrade = hou.ButtonParmTemplate('update_strokes','Migrate Old Brush Data',
        help='Update strokes created by an older KoAdjust while keeping their anchors and settings. Usually automatic.')
    upgrade.setScriptCallback('import ko_adjust\nko_adjust.update_saved_strokes(kwargs["node"])')
    upgrade.setScriptCallbackLanguage(hou.scriptLanguage.Python)
    ptg.append(upgrade)
    ptg.append(hou.StringParmTemplate('view_hidden','Temporary Hidden Parts',1,is_hidden=True))
    ptg.append(hou.StringParmTemplate('view_focus','Temporary Editing Parts',1,is_hidden=True))
    ptg.append(hou.StringParmTemplate('editing_parts_data','Brush Target',1,is_hidden=True))
    definition.setParmTemplateGroup(ptg)
    assert node.node('stack').isEditableInsideLockedHDA()


if __name__ == '__main__':
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / DEST.name
        build(path)
        DEST.write_bytes(path.read_bytes())
    print(DEST)
