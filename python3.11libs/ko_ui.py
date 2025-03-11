import hou
from collections.abc import Callable

def reset_viewer_state(viewer):
    if viewer.currentState() != 'sopview':
        viewer.enterViewState()
        viewer.enterCurrentNodeState()

def setShadingMode(shading_type: hou.glShadingType, display_set_type: hou.displaySetType = hou.displaySetType.DisplayModel):
    # pane = hou.ui.curDesktop().paneTabOfType(hou.paneTabType.SceneViewer)
    paneTab = hou.ui.curDesktop().paneTabUnderCursor()

    settings = paneTab.curViewport().settings()
    tmplset = settings.displaySet(display_set_type)
    tmplset.setShadedMode(shading_type)

def menuize(items: list[str], itemToLabel: Callable[[str], str] = lambda item: item, with_empty: bool = False) -> list[str]:
    """
    Convert a list of strings to a list of alternating labels and items.
    """
    labels = [itemToLabel(item) for item in items]
    if with_empty:
        items = [" "] + items
        labels = ["None"] + labels
    return [item for pair in zip(labels, items) for item in pair]

# multiparm index is 1-based
def currentMultiparmIndex(parm: hou.Parm) -> int:
    """
    Get the current multiparm index of a multiparm instance.
    """
    if not parm.isMultiParmInstance():
        raise Exception("Parm is not a multiparm instance")
    return parm.multiParmInstanceIndices()[0]

def currentParm() -> hou.Parm:
    return hou.pwd().parm( hou.expandString('$CH') ) 

def siblingMultiParm(name: str, component_index: int | None = None, current_parm: hou.Parm = None) -> hou.Parm:
    """
    Get the multiparm instance of a sibling multiparm.
    """
    parm = current_parm if current_parm else currentParm() 
    index = currentMultiparmIndex(parm)
    component_index = component_index if component_index != None else parm.componentIndex()
    return hou.parmTuple(name+str(index))[component_index]

def evalSiblingMultiParm(name: str) -> int:
    """
    Evaluate a sibling multiparm instance.
    """
    parm = siblingMultiParm(name)
    return parm.eval()


# https://forums.odforce.net/topic/44450-multiparm-block-change-the-order/
def swapMultiParmValues(kwargs, a: str, b: str):
    #a, b are parameter names (not labels)
    node = kwargs["node"]
    pA = node.parm(a)
    pB = node.parm(b)
    templateGroup = node.parmTemplateGroup()
    template = pA.parmTemplate().clone()
    template.setName("_temp_")
    templateGroup.append(template)
    node.setParmTemplateGroup(templateGroup)
    pTemp = node.parm("_temp_")
    pTemp.setFromParm(pA)
    pA.setFromParm(pB)
    pB.setFromParm(pTemp)
    templateGroup.remove(template)
    node.setParmTemplateGroup(templateGroup)


def getParamNames(kwargs, mpBlock: hou.ParmTemplate, index: int, swapIndex: int, nestingDepth: int):
    node = kwargs["node"]
    
    for i in range(len(mpBlock)):
        #If the current parameter is of a valid type, check if it has channels
        if mpBlock[i].type() == hou.parmTemplateType.Int or mpBlock[i].type() == hou.parmTemplateType.Float or mpBlock[i].type() == hou.parmTemplateType.String or mpBlock[i].type() == hou.parmTemplateType.Toggle:
            
            #note that vector channels are suffixed after multiparm index - "vector_#x" instead of "vector_x#"
            if mpBlock[i].numComponents() > 1:
                for c in range(mpBlock[i].numComponents()):
                    if mpBlock[i].namingScheme() == hou.parmNamingScheme.XYZW:
                        if c == 0:
                            vComponent = "x"
                        elif c == 1:
                            vComponent = "y"
                        elif c == 2:
                            vComponent = "z"
                        elif c == 3:
                            vComponent = "w"
                    elif mpBlock[i].namingScheme() == hou.parmNamingScheme.RGBA:
                        if c == 0:
                            vComponent = "r"
                        elif c == 1:
                            vComponent = "g"
                        elif c == 2:
                            vComponent = "b"
                        elif c == 3:
                            vComponent = "a"
                    elif mpBlock[i].namingScheme() == hou.parmNamingScheme.UVW:
                        if c == 0:
                            vComponent = "u"
                        elif c == 1:
                            vComponent = "v"
                        elif c == 2:
                            vComponent = "w"
                    pName = mpBlock[i].name().replace("#","%s") % index + vComponent
                    pOthrName = mpBlock[i].name().replace("#","%s") % (index+swapIndex) + vComponent
            
                    swapMultiParmValues(kwargs, pName, pOthrName)
                    
            else:
                pName = mpBlock[i].name().replace("#","%s") % index
                pOthrName = mpBlock[i].name().replace("#","%s") % (index+swapIndex)
                swapMultiParmValues(kwargs, pName, pOthrName)
                
        #if a folder is found, determine if it's a nested multiparm
        elif mpBlock[i].type() == hou.parmTemplateType.Folder:
            #if it is, compare the number of instances in each multiparm
            if mpBlock[i].folderType() == hou.folderType.MultiparmBlock:
                
                getNMP = mpBlock[i].name().replace("#","%s") % index
                getOthrNMP = mpBlock[i].name().replace("#","%s") % (index+swapIndex)
                nmpInstances = node.parm(getNMP).evalAsInt()
                nmpOthrInstances = node.parm(getOthrNMP).evalAsInt()
                
                #If both multiparms have the same number of instances, swap nested parameter values
                if nmpInstances == nmpOthrInstances:
                    for j in range(nmpInstances):
                        pA = node.parm(getNMP).parmTemplate().parmTemplates()[j-1].name().replace("#","%s") % (index, j+1)
                        pB = node.parm(getOthrNMP).parmTemplate().parmTemplates()[j-1].name().replace("#","%s") % (index+swapIndex, j+1)
                        swapMultiParmValues(kwargs, pA, pB)
                #Otherwise, save values to a temporary holder
                else:
                    tempA = list()
                    tempB = list()
                    for j in range(nmpInstances):
                        nestedParm = node.parm(getNMP).parmTemplate().parmTemplates()[j-1].name().replace("#","%s") % (index, j+1)
                        if len(node.parm(nestedParm).keyframes()) > 0:
                            tempA.append(node.parm(nestedParm).keyframes())
                        else:
                            tempA.append(node.parm(nestedParm).rawValue())
                        
                    for j in range(nmpOthrInstances):
                        nestedParm = node.parm(getOthrNMP).parmTemplate().parmTemplates()[j-1].name().replace("#","%s") % (index+swapIndex, j+1)
                        if len(node.parm(nestedParm).keyframes()) > 0:
                            tempB.append(node.parm(nestedParm).keyframes())
                        else:
                            tempB.append(node.parm(nestedParm).rawValue())
                            
                    #initialize number of multiparm blocks
                    swapMultiParmValues(kwargs, getNMP, getOthrNMP)
                    
                    #and update each block from the temporary holders
                    for k in range(nmpOthrInstances):
                        pA = node.parm(getNMP).parmTemplate().parmTemplates()[k-1].name().replace("#","%s") % (index, k+1)
                        node.parm(pA).deleteAllKeyframes()
                        try:
                            node.parm(pA).set(tempB[k])
                        except:
                            node.parm(pA).setKeyframes(tempB[k])
                    for k in range(nmpInstances):
                        pB = node.parm(getOthrNMP).parmTemplate().parmTemplates()[k-1].name().replace("#","%s") % (index+swapIndex, k+1)
                        node.parm(pB).deleteAllKeyframes()
                        try:
                            node.parm(pB).set(tempA[k])
                        except:
                            node.parm(pB).setKeyframes(tempA[k])
                
            #if it's not a multiparm, dive inside and swap each nested parameter
            else:
                getParamNames(kwargs, mpBlock[i].parmTemplates(), index, swapIndex, 0)

def swapMultiParmTo(kwargs, targetSwapIndex: int):
    node = kwargs["node"]
    button = kwargs["parm"]
    
    #Shorthand to access the index of a multiparm
    index = int(kwargs["script_multiparm_index"])
    
    #Raise error if parameter hierarchy is configured incorrectly
    if not button.tuple().isMultiParmInstance():
        raise hou.NodeWarning("Button is not inside a multiparm block.")        
    
    #Get the parent multiparm folder
    mpFolder = button.tuple().parentMultiParm()
    
    #Count the number of multiparm instances -> raise errors if swapping is not allowed
    mpInstances = node.parm(mpFolder.name()).evalAsInt()
    
    #Raise errors if trying to swap up on first block, or swap down on last block
    if targetSwapIndex > 0:
        if index == mpInstances:
            raise hou.NodeWarning("No value below to swap with.")
    elif targetSwapIndex < 0:
        if index == 1:
            raise hou.NodeWarning("No value above to swap with.")
    
    #Get the other parameters inside this multiparm block so we can start swapping.
    mpBlock = node.parm(mpFolder.name()).parmTemplate().parmTemplates()
    getParamNames(kwargs, mpBlock, index, targetSwapIndex, 0)


def allActiveNetworkEditors():
    return [pane for pane in hou.ui.paneTabs() if isinstance(pane, hou.NetworkEditor) and pane.isCurrentTab()]

def oneNetworkEditor(kwargs):
    if 'pane' in kwargs and isinstance(kwargs['pane'], hou.NetworkEditor):
        return kwargs['pane']
    return allActiveNetworkEditors()[-1]