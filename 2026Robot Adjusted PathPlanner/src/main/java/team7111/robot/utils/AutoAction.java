package team7111.robot.utils;

import java.util.function.BooleanSupplier;

import team7111.lib.pathfinding.Path;
import team7111.robot.subsystems.SuperStructure.SuperState;

public class AutoAction {
    private Path path = null;
    private SuperState state = null;
    private boolean isPath = false;
    private boolean isState = false;
    private BooleanSupplier alternateCondition = null;
    private BooleanSupplier additionalCondition = null;
    private BooleanSupplier optionalCondition = null;
    private boolean hasAlternateCondition = false;
    private boolean hasAdditionalCondition = false;
    private boolean hasOptionalCondition = false;

    /**
     * Constructs the object to contain a Path. Use AutoAction(SuperState) to contain a SuperState
     * @param state The object to contain as a SuperState
     */
    public AutoAction(Path path){
        this.path = path;
        isPath = true;
    }
    /**
     * Constructs the object to contain a SuperState. Use AutoAction(Path) to contain a Path
     * @param state The object to contain as a SuperState
     */
    public AutoAction(SuperState state){
        this.state = state;
        isState = true;
    }

    // Getter methods go below here

    public SuperState getAsState(){
        if(state != null){
            return state;
        }
        throw new NullPointerException("Instance of AutoAction does not contain SuperState!\nCheck constructor used for object instantiation");
    }

    public Path getAsPath(){
        if(path != null){
            return path;
        }
        throw new NullPointerException("Instance of AutoAction does not contain Path!\nCheck constructor used for object instantiation.");
    }

    public boolean getAlternateCondition(){
        return alternateCondition.getAsBoolean();
    }

    public boolean getAdditionalCondition(){
        return additionalCondition.getAsBoolean();
    }

    public boolean getOptionalCondition(){
        return optionalCondition.getAsBoolean();
    }

    // Condition methods go below here

    public boolean isPath(){
        return isPath;
    }

    public boolean isState(){
        return isState;
    }

    public boolean hasAlternateCondition(){
        return hasAlternateCondition;
    }

    public boolean hasAdditionalCondition(){
        return hasAdditionalCondition;
    }

    public boolean hasOptionalCondition(){
        return hasOptionalCondition;
    }

    // these are called chain methods. When you instatiate an object, you can call these to change additional information

    /**
     * Decorates the AutoAction with an alternate condition. This will act as a replacement to the previous condition
     * @param condition the new condition of the AutoAction
     * @return itself for method chaining
     */
    public AutoAction withAlternateCondition(BooleanSupplier condition){
        alternateCondition = condition;
        hasAlternateCondition = true;
        hasAdditionalCondition = false;
        hasOptionalCondition = false;
        return this;
    }

    /**
     * Decorates the AutoAction with an additional condition. This will act as an AND statement with the default condition
     * @param condition the additional condition to be compared
     * @return itself for method chaining
     */
    public AutoAction withAdditionalCondition(BooleanSupplier condition){
        additionalCondition = condition;
        hasAdditionalCondition = true;
        return this;
    }

    /**
     * Decorates the AutoAction with an optional condition. This will act as an OR statement with the default condition
     * @param condition the optional condition to be comnpared
     * @return itself for method chaining
     */
    public AutoAction withOptionalCondition(BooleanSupplier condition){
        optionalCondition = condition;
        hasOptionalCondition = true;
        return this;
    }

    /**
     * Useful shorthand for decorating the AutoAction with no condition. This is the same as solely calling withAlternateCondition(() -> true)
     * @return itself for method chaining
     */
    public AutoAction withNoConditions(){
        withAlternateCondition(() -> true);
        return this;
    }
}
